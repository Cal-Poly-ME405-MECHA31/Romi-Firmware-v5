import micropython
from time import ticks_us, ticks_diff
from pyb import Pin, Timer, UART, ExtInt, I2C
from array import array
import task_share
import cotask
import struct

## PID is an generic, abstract PID class used in various places in Romi's firmware.  It is used to store constants, and when
#  an output is needed the function calculate(error) is called.  This class also features integral saturation: if the integral
#  effort reaches an assigned threshold, the integral stops growing.  In addition, the class also features the ability to average
#  the last several derivative values for some basic filtering.
class PID: # abstract PID class

	def __init__(self, K_P, K_I, K_D, intLim, derAve):
		self.K_P = K_P
		self.K_I = K_I
		self.K_D = K_D
		self.intLim = intLim # integral limit.  When the integral effort reaches this value, the integral stops growing
		self.derAve = derAve # number of derivative values to average out when calculating derivative effort
		self.integral = 0
		self.prevError = 0
		self.timeStamp = ticks_us()
		self.der_list = array('i', derAve*[0]) # der list contains the most recent derivative effort values for averaging
		self.der_index = 0

	## Calling zero assigns the integral to 0 immediately.  This is used to reset the controller
	def zero(self):
		self.integral = 0

	## Calling setTimeStamp assigns the timestamp that is used for derivative calculation immediately.  This is used to reset the controller
	def setTimeStamp(self):
		self.timeStamp = ticks_us()

	## In some cases, the firmware possesses the ability to dynamically alter the PID constants with user input.  This function allows the quick reassignment of the constants.
	def setPID(self, K_P, K_I, K_D):
		self.K_P = K_P
		self.K_I = K_I
		self.K_D = K_D

	## calculate is the main computation of the PID controller.  Calculate takes a unitless input, error, and returns a unitless output effort
	def calculate(self, error):
		deltaT = ticks_diff(ticks_us(), self.timeStamp)
		self.timeStamp = ticks_us()

		proEffort = self.K_P * error # proportional effort

		intEffort = 0 # integral effort
		if self.K_I > 0:
			self.integral += error * deltaT
			intEffort = self.K_I * self.integral / 1000000 # integral value is converted from microseconds
			if self.intLim > 0:
				if abs(intEffort) > self.intLim:
					self.integral -= error * deltaT # if integral effort is too high, undo the effects on the integral
					if intEffort > 0:
						intEffort = self.intLim
					else:
						intEffort = -1*self.intLim
		else:
			self.integral = 0

		derEffort = 0
		if self.K_D > 0:
			if deltaT > 0:
				self.der_list[self.der_index] = int(self.K_D * (error - self.prevError) / deltaT) # assign the next derivative in the list
				if self.der_index == self.derAve-1:
					self.der_index = 0 # reset the index
				else:
					self.der_index += 1 # increment the index
				der_total = 0
				for i in range(self.derAve):
					der_total += self.der_list[i]
				if der_total > 0:
					derEffort = int(der_total / self.derAve)
			self.prevError = error

		return int(proEffort + intEffort + derEffort)

	## @var K_P
    #  Current proportional constant

    ## @var K_I
    #  Current integral constant

    ## @var K_D
    #  Current derivative constant

    ## @var intLim
    #  The maximum integral effort output value before the integral terms stops growing

    ## @var derAve
    #  The number of past derivative terms to average out when calculating next output

    ## @var prevError
    #  The error from the last calculation cycle for derivative calculations

    ## @var timeStamp
    #  Timestamp used for derivative calculations

    ## @var der_list
    #  List of ints containing the last several derivative effort outputs for averaging

    ## @var der_index
    #  Index of the last place in der_list that was assigned a value