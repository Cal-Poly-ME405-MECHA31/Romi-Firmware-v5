import micropython
from time import ticks_us, ticks_diff
from pyb import Pin, Timer, UART, ExtInt, I2C
from array import array
import task_share
import cotask
import struct

## The Motor task is a simple motor driver.  It allows enable/disable as well as setting the current effort.
class Motor:

	MAX_EFFORT = 100

	def __init__(self, tim, chan, PWM, DIR, nSLP):
		timer = Timer(tim, freq=20000)
		self.PWM_pin = timer.channel(chan, pin=PWM, mode=Timer.PWM, pulse_width_percent=0)
		self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP, value=0)
		self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)

	def set_effort(self, effort):
		if effort < 0:
			self.DIR_pin.high()
		else:
			self.DIR_pin.low()

		if abs(effort < Motor.MAX_EFFORT):
			self.PWM_pin.pulse_width_percent(abs(effort))
		else:
			self.PWM_pin.pulse_width_percent(Motor.MAX_EFFORT)

	def enable(self):
		self.nSLP_pin.high()

	def disable(self):
		self.nSLP_pin.low()

	## @var PWM_pin
    #  Timer channel object for controlling motor effort

    ## @var DIR_pin
    #  Pin object for assigning motor direction

    ## @var nSLP_pin
    #  Pin object for enabling or disabling motor.  High = enabled, low = disabled

    ## @var MAX_EFFORT
    #  Saturation value for allowable effort