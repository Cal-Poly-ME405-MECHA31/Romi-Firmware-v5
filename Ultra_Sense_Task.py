from time import ticks_us, ticks_diff, sleep_us, sleep_ms
from pyb import Pin, Timer, UART, ExtInt, I2C
from array import array
import task_share
import cotask
import struct
from gc import collect

## Ultra_Sense_Task provides a driver to track the various ultrasonic sensors.  The ultrasonic sensors are the HC-SR04 model.
#  The steps to read from each sensor are 1) start with the trigger pin low 2) drive the trigger pin high briefly 3) enable 
#  the  echo pin callback  4) wait for the pin to be driven low by the sensor 6) record the time.  
#  Every call to the run() function, we read, at most, one sensor, if it is available.
class Ultra_Sense_Task:

	## Number of ultrasonic sensor read by this object
	SENSOR_COUNT = 2

	def __init__(self, dataQueue, ctrlState, distance):
		self.dataQueue = dataQueue
		self.ctrlState = ctrlState

		self.distance = []
		for i in range(Ultra_Sense_Task.SENSOR_COUNT):
			self.distance.append(distance[i])

		self.state = 0


	## callback(pin) defines the callback when the ultrasonic echo sensor pin is driven low by the sensor to indicate the distance.  Data is stored directly in the 
	#  rawData variable because floating point math is forbidden in callbacks
	def callback(self, pin): 
		self.rawData[self.index] = ticks_diff(ticks_us(), self.timeStamp) 

		if Ultra_Sense_Task.SENSOR_COUNT > 1:

			self.interruptPins[self.index].disable() 

			if self.index >= Ultra_Sense_Task.SENSOR_COUNT-1:
				self.index = 0 # reset index
			else:
				self.index += 1 # increment index

		else:

			self.index = -1

	## run() runs the next state of the state machine.  It is intended to be called by the scheduler with a period of 10ms.
	#  Once in the reading state, every call to run() does the following things: 1) enable the callback for the 
	#  current sensor 3) pulse the trigger pin 4) update distance
	def run(self):

		while True:

			if self.state == 0: # state 0 is only run once.  State 0 initializes hardware objects

				self.echoNums = [Pin.cpu.B2, Pin.cpu.B14]
				# self.echoNums = [Pin.cpu.B2]
				self.index = 0

				self.rawData = []
				for i in range(len(self.echoNums)):
					self.rawData.append(0)

				self.interruptPins = []
				for num in self.echoNums:
					self.interruptPins.append(ExtInt(num, ExtInt.IRQ_FALLING, Pin.PULL_NONE, self.callback))
					if Ultra_Sense_Task.SENSOR_COUNT > 1:
						self.interruptPins[-1].disable()

				self.trigNums = [Pin.cpu.B11, Pin.cpu.B13]
				# self.trigNums = [Pin.cpu.B11]
				self.trigPins = []
				for num in self.trigNums:
					self.trigPins.append(Pin(num, mode=Pin.OUT_PP, value=0))

				self.state = 1

			elif self.state == 1:

				if (self.ctrlState.get() == 6) or (self.ctrlState.get() == 4): # wait for ctrlState to be set to 2
					self.state = 2

			elif self.state == 2: # state 2 is run every time the sensors are turned off and on again

				self.index = 0
				self.prevIndex = -1
				self.rawData = []
				for i in range(len(self.echoNums)):
					self.rawData.append(0)	

				self.state = 3

			elif self.state == 3: # IR sensors on state

				if self.ctrlState.get() not in [4, 6]: # check if ctrlState indicates sensors should be on
					self.state = 1
					continue

				if self.index != self.prevIndex: # if index has changed, it means the callback has run

					if self.index == -1:
						self.index = 0

					self.timeStamp = ticks_us()

					if Ultra_Sense_Task.SENSOR_COUNT > 1:
						self.interruptPins[self.index].enable() # enable the callback when the sensor drives the line high again

					self.trigPins[self.index].high()
					sleep_us(20)
					self.trigPins[self.index].low()

					self.prevIndex = self.index

				if ticks_diff(ticks_us(), self.timeStamp) > 1000000: # this gets called if one of the sensors doesn't respond to the trigger fast enough

					if Ultra_Sense_Task.SENSOR_COUNT > 1:

						self.interruptPins[self.index].disable() # disable the current pin so it doesn't get called again

						if self.index >= Ultra_Sense_Task.SENSOR_COUNT-1:
							self.index = 0 # reset index
						else:
							self.index += 1 # increment index

					else:

						self.index = -1

				for i in range(Ultra_Sense_Task.SENSOR_COUNT):
					self.distance[i].put(self.rawData[i])

				if self.ctrlState.get() == 6:
					if Ultra_Sense_Task.SENSOR_COUNT < 2:
						print("ULTRA SENSE: Distance: " + str(self.distance[0].get()))
					else:
						print("ULTRA SENSE: Distance 1: " + str(self.distance[0].get()) + ", Distance 2: " + str(self.distance[1].get()))

			else:
				self.state = 0

			yield self.state

	## @var state
    #  Next state to be run by the state machine.  0 = initialization, 1 = waiting, 2 = reset, 3 = sensing

    ## @var index
    #  Index in list of the current sensor being read from the array

    ## @var echoNums
    #  List of the pin identifier objects associated with each echo pin

    ## @var rawData
    #  List of most recent measurement of reflectance directly from the sensor, expressed as microseconds of delay

    ## @var interruptPins
    #  List of ExtInt objects for enabling and disabling the interrupt callabacks

    ## @var trigNums
    #  List of the pin identifier objects associated with each trigger pin

    ## @var trigPins
    #  List of Pin objects associated with each trigger pin

    ## @var prevIndex
    #  The index of the last sensor that was read, to identify when the callback has completed its reading

    ## @var timeStemp
    #  Time stamp for determining if the sensor has timed out