import micropython
from pyb import Pin, Timer, UART, ExtInt, I2C
from time import ticks_us, ticks_diff
from array import array
from gc import collect
import struct
import task_share
import cotask
import math

class XY_Tracking:

	def __init__(self, encoderPosition, dataQueue, ctrlState, heading, calibrated, xPosition, yPosition):

		self.encoderPosition = []
		self.encoderPosition.append(encoderPosition[0])
		self.encoderPosition.append(encoderPosition[1])

		self.dataQueue = dataQueue

		self.ctrlState = ctrlState

		self.heading = heading

		self.calibrated = calibrated

		self.xPosition = xPosition
		self.yPosition = yPosition

		self.state = 0

	def run(self):

		while True:

			if self.state == 0:

				self.prevPosition = []
				self.prevPosition.append(self.encoderPosition[0].get())
				self.prevPosition.append(self.encoderPosition[1].get())

				self.state = 1

			elif self.state == 1:

				if self.ctrlState.get() == 4:

					if self.calibrated.get() == 1:

						self.xPosition.put(0.0)
						self.yPosition.put(0.0)

						self.startHeading = self.heading.get()

						self.timeStamp = ticks_us()

						self.state = 2

			elif self.state == 2:

				if self.ctrlState.get() != 4:

					self.state = 1

				else:

					positionDelta = ((self.encoderPosition[0].get() - self.prevPosition[0]) + (self.encoderPosition[1].get() - self.prevPosition[1])) / 2 * (1 / 1440 * 2*3.14159 * 0.035)

					theta = self.heading.get() - self.startHeading
					if theta < 0:
						theta += 360
					elif theta > 360:
						theta -= 360

					self.xPosition.put(self.xPosition.get() - positionDelta*math.cos(theta*3.14159/180))
					self.yPosition.put(self.yPosition.get() + positionDelta*math.sin(theta*3.14159/180))

					self.prevPosition[0] = self.encoderPosition[0].get()
					self.prevPosition[1] = self.encoderPosition[1].get()

					if ticks_diff(ticks_us(), self.timeStamp) > 1000000:

						self.dataQueue.put(-0.1)
						self.dataQueue.put(-0.1)
						self.dataQueue.put(1)
						self.dataQueue.put(self.xPosition.get())
						self.dataQueue.put(self.yPosition.get())

						self.timeStamp = ticks_us()

			else:
				self.state = 0

			yield self.state