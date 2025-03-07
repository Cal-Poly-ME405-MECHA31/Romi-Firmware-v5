import micropython
from time import ticks_us, ticks_diff
from pyb import Pin, Timer, UART, ExtInt, I2C
from array import array
import task_share
import cotask
import struct

## IR_Sense_Task provides a driver to track the array of IR sensors for line following.  The IR sensors are the RC model, not the analog model.
#  The steps to read from each sensor are 1) start with the pin low 2) drive the pin high briefly 3) enable the pin callback 
#  4) assign the pin as an input to disconnect 5) wait for the pin to be driven high by the sensor 6) record the time.  
#  Every call to the run() function, we read, at most, one sensor, if it is available.
class IR_Sense_Task:

	## The number of IR reflectance strength values to average over time before assigning strength
	STRENGTH_AVE = 5
	## The largest allowable offset reading before we begin to saturate
	MAX_OFFSET = 400 
	## Defines an automatic bias in the offset value
	BIAS = 0

	def __init__(self, dataQueue, ctrlState, offset, strength):
		self.dataQueue = dataQueue
		self.ctrlState = ctrlState

		self.offset = offset
		self.strength = strength

		self.state = 0

	## callback(pin) defines the callback when the IR sensor pin is driven low by the sensor to indicate the reflectance.  Data is stored directly in the 
	#  rawData variable because floating point math is forbidden in callbacks
	def callback(self, pin):

		self.rawData[self.index] = ticks_diff(ticks_us(), self.timeStamp)
		self.interruptPins[self.index].disable() # disable the current pin so it doesn't get called again
		if self.index >= len(self.pinNums)-1:
			self.index = 0 # reset index
		else:
			self.index += 1 # increment index

	## run() runs the next state of the state machine.  It is intended to be called by the scheduler with a period of 2ms.
	#  Once in the reading state, every call to run() does the following things: 1) Set the next sensor high 2) enable the 
	#  callback for the current sensor 3) set the current sensor to an input 4) update strength and offset
	def run(self):

		while True:

			if self.state == 0: # state 0 is only run once.  State 0 initializes hardware objects

				self.pinNums = [Pin.cpu.C10, Pin.cpu.C12, Pin.cpu.A15, Pin.cpu.H0, Pin.cpu.H1, Pin.cpu.C3, Pin.cpu.C11]
				self.pinLines = [10, 12, 15, 0, 1, 3, 11]
				self.referenced = 0
				self.index = 0
				self.index2 = int(len(self.pinNums)/2)

				self.rawData = []
				for i in range(len(self.pinNums)):
					self.rawData.append(0)

				self.pinList = []
				self.interruptPins = []
				for num in self.pinNums:
					self.interruptPins.append(ExtInt(num, ExtInt.IRQ_FALLING, Pin.PULL_NONE, self.callback))
					self.interruptPins[-1].disable()
					self.pinList.append(Pin(num, mode=Pin.OUT_PP, value=0))

				self.state = 1

			elif self.state == 1:

				if self.ctrlState.get() == 2: # wait for ctrlState to be set to 2
					self.state = 2

			elif self.state == 2: # state 2 is run every time the sensors are turned off and on again

				self.index = 0
				self.index2 = int(len(self.pinNums)/2)
				self.prevIndex = -1
				self.prevIndex2 = -1
				self.rawData = []
				self.reflectance = []
				self.whiteRef = []
				self.blackRef = []
				for i in range(len(self.pinNums)):
					self.rawData.append(0)
					self.reflectance.append(0)
					self.whiteRef.append(0)
					self.blackRef.append(0)

				self.strength_list = array('i', IR_Sense_Task.STRENGTH_AVE*[0])
				self.strength_index = 0
				self.pinList[0].high()

				self.state = 3

			elif self.state == 3: # IR sensors on state

				if self.ctrlState.get() not in [2, 3, 4]: # check if ctrlState indicates sensors should be on
					self.state = 1
					continue

				if self.index != self.prevIndex: # if index has changed, it means the callback has run

					if self.index >= len(self.pinList)-1:
						self.pinList[0].init(Pin.OUT_PP) # if we're at the end of the array, prep the first sensor
						self.pinList[0].high()
					else:
						self.pinList[self.index+1].init(Pin.OUT_PP) # if were not at the end of the array, prep the next sensor by driving the line high
						self.pinList[self.index+1].high()

					self.timeStamp = ticks_us()
					self.line1 = self.pinLines[self.index]
					self.interruptPins[self.index].enable() # enable the callback when the sensor drives the line high again

					self.pinList[self.index].init(Pin.IN) # trigger the current pin by disconnecting the pin
					
					self.prevIndex = self.index


				if ticks_diff(ticks_us(), self.timeStamp) > 10000: # this gets called if one of the sensors doesn't respond to the trigger fast enough
					if self.index == self.prevIndex:
						self.interruptPins[self.index].disable()
						if self.index >= len(self.pinNums)-1:
							self.index = 0
						else:
							self.index += 1
						

				if self.ctrlState.get() == 3: # ctrlState of 3 indiccates IR reference
					if self.referenced == 0: # the first press is for the white reference
						for i,value in enumerate(self.rawData):
							self.whiteRef[i] = value
						self.referenced = 1

					elif self.referenced == 1: # the second press is for the black reference
						for i,value in enumerate(self.rawData):
							self.blackRef[i] = value
						self.referenced = 2

					elif self.referenced == 2: # a third press resets the references
						self.referenced = 0

					self.dataQueue.put(-0.1)
					self.dataQueue.put(-0.1)
					self.dataQueue.put(1)
					self.dataQueue.put(self.referenced) # print to the bluetooth to indicate completion to user

					self.ctrlState.put(2)

				if self.referenced == 2: # if both references have been set, calculate the reflectance accordingly
					for i,value in enumerate(self.rawData):
						self.reflectance[i] = abs((self.rawData[i] - self.whiteRef[i]) / (self.blackRef[i] - self.whiteRef[i]) * 100)
				else: # if references haven't been set, just copy the raw data over
					for i,value in enumerate(self.rawData):
						self.reflectance[i] = self.rawData[i]

				weight = 0
				total = 0
				for i,value in enumerate(self.reflectance):
					weight += (i - int(len(self.pinNums)/2)) * value # weight is index * reflectance to calculate centroid
					total += value
				nextOffset = weight / total * 100 if total else 0 # offset is measured in [IR indexes * 100] to keep the value as an int
				if nextOffset > IR_Sense_Task.MAX_OFFSET: # apply saturation to offset to filter out extraneous results
					nextOffset = IR_Sense_Task.MAX_OFFSET
				elif nextOffset < -1*IR_Sense_Task.MAX_OFFSET:
					nextOffset = -1*IR_Sense_Task.MAX_OFFSET
				self.offset.put(int(nextOffset) + IR_Sense_Task.BIAS)

				self.strength_list[self.strength_index] = abs(int(total / len(self.reflectance) * 10)) # strength is average reflectance * 10 to keep the value as an int
				if self.strength_index == IR_Sense_Task.STRENGTH_AVE-1:
					self.strength_index = 0
				else:
					self.strength_index += 1
				strength_total = 0
				for i in range(IR_Sense_Task.STRENGTH_AVE):
					strength_total += self.strength_list[i]
				if strength_total > 0:
					self.strength.put(int(strength_total / IR_Sense_Task.STRENGTH_AVE)) # strength is averaged over both space and time

				if self.ctrlState.get() == 2:
					print("IR TASK: Offset: " + str(self.offset.get()) + ", Strength: " + str(self.strength.get()) + ", Reflectances: " + str(self.reflectance))

			else:
				self.state = 0

			yield self.state

	## @var leftMotor
    #  Motor object for setting left motor parameters