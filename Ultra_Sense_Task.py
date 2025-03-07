from time import ticks_us, ticks_diff, sleep_us, sleep_ms
from pyb import Pin, Timer, UART, ExtInt, I2C
from array import array
import task_share
import cotask
import struct
from gc import collect

class Ultra_Sense_Task: # driver to read from the IR sensors

	SENSOR_COUNT = 2

	def __init__(self, dataQueue, ctrlState, distance):
		self.dataQueue = dataQueue
		self.ctrlState = ctrlState

		self.distance = []
		for i in range(Ultra_Sense_Task.SENSOR_COUNT):
			self.distance.append(distance[i])

		self.state = 0

	def callback(self, pin): # function is called when the IR sensor drives the pin high to communicate the reflectance
		self.rawData[self.index] = ticks_diff(ticks_us(), self.timeStamp) # raw data is stored here to avoid floating point calcs in the callback

		if Ultra_Sense_Task.SENSOR_COUNT > 1:

			self.interruptPins[self.index].disable() # disable the current pin so it doesn't get called again

			if self.index >= Ultra_Sense_Task.SENSOR_COUNT-1:
				self.index = 0 # reset index
			else:
				self.index += 1 # increment index

		else:

			self.index = -1


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