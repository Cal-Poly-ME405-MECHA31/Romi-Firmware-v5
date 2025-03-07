import micropython
from time import ticks_us, ticks_diff
from pyb import Pin, Timer, UART, ExtInt, I2C
from array import array
import task_share
import cotask
import struct

## The Encoder class defines a simple Encoder driver by providing access to encoder position and velocity.  Note:
#  the Encoder class is a wrapper not a task: to function properly, regular calls to the update() function must be
#  included in logic elsewhere
class Encoder:

	## The number of past velocity terms to average out when current velocity is requested
 	VEL_AVG = 5

	def __init__(self, tim, chA_pin, chB_pin):
		self.position = 0 # calculated position (corrected for overflow)
		self.count = 0 # measured count (not corrected for overflow)
		self.prev_count = 0
		self.delta = 0
		self.dt = 0
		self.last_ts = 0
		self.vel_list = array('f', Encoder.VEL_AVG*[0]) # list of most recent velocity values to average out
		self.vel_index = 0

		self.encTimer = Timer(tim, period = 0xFFFF, prescaler = 0)
		self.encTimer.channel(1, pin=chA_pin, mode=Timer.ENC_AB)
		self.encTimer.channel(2, pin=chB_pin, mode=Timer.ENC_AB)

	## update() is the function that must be called regularly for encoder values to update.  Recommended period is 5ms
	def update(self):
		ts = ticks_us()
		self.dt = ticks_diff(ts, self.last_ts)
		self.last_ts = ts

		self.prev_count = self.count
		self.count = self.encTimer.counter()

	 	if abs(self.prev_count - self.count) > (65536/2): # if the change was too large, correct for encoder overflow:
	 		if self.count - self.prev_count > (65536/2):
	 			self.delta = self.count - self.prev_count - 65536
			elif self.count - self.prev_count < -1*(65536/2):
				self.delta = self.count - self.prev_count + 65536
		else:
			self.delta = self.count - self.prev_count

		self.position -= self.delta

		if self.vel_index == Encoder.VEL_AVG-1:
			self.vel_index = 0
		else:
			self.vel_index += 1

		self.vel_list[self.vel_index] = -1*self.delta/self.dt*1000000 if self.dt else 0 # encoder velocity is in ticks/s

	def get_position(self):
		return self.position

	## get_velocity averages the last VEL_AVG data points
	def get_velocity(self):
		vel_total = 0
		for i in range(Encoder.VEL_AVG): # velocity is only averaged when requested
			vel_total += self.vel_list[i]
		return vel_total / Encoder.VEL_AVG

	## @var position
    #  Current encoder position, corrected for overflow and underflow

    ## @var count
    #  Raw count read directly from Timer object (not corrected for overflow/underflow)

    ## @var prev_count
    #  Raw count read from the last call to update

    ## @var delta
    #  The change in position/count from last update to current update

    ## @var dt
    #  The change in time from last update to current update

    ## @var last_ts
    #  Timestap of the last call to update

    ## @var vel_list
    # List containing the velocity at the last several calls to update for averaging

    ## @var vel_index
    #  Current index associated with the vel_list object

    ## @var encTimer
    #  Timer object associated with each encoder.  The Timer object automatically updates the count


