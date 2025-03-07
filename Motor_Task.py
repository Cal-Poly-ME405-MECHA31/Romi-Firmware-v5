import micropython
from time import ticks_us, ticks_diff
from pyb import Pin, Timer, UART, ExtInt, I2C
from array import array
import task_share
import cotask
import struct

from PID import PID

## Motor_Task defines a state machine for position and velocity control of the motor.  There is one Motor_Task for each motor
#  encoder pair.  Motor_Task will automatically switch to position or velocity control every time the position or velocity 
#  targets are updated, respectively
class Motor_Task:

	## Maximum effort that the PID loop will command to the motor
	MAX_EFFORT = 75

	## K_P for position control
	POS_P = 1
	## dead band for position control
	POS_DB = 10 
	## max position control effort
	POS_MAX = 25 

	## K_P for velocity control
	VEL_P = 0.1 
	## K_I for velocity control
	VEL_I = 0
	## K_D for velocity control
	VEL_D = 0 
	## Number of derivative values to average
	DER_AVE = 10 
	## feed forward gain, in percent effort / tick/s
	VEL_FF = 0.02 

	def __init__(self, chiralIndex, motor, encoder, encoderPosition, targetPosition, targetVelocity):
		self.chiralIndex = chiralIndex # 0 = left, 1 = right

		self.motor = motor
		self.encoder = encoder

		self.encoderPosition = encoderPosition[chiralIndex]
		self.targetPosition = targetPosition[chiralIndex]
		self.targetVelocity = targetVelocity[chiralIndex]

		self.state = 0

	## run() runs the next state of the state machine.  It is intended to be called by the scheduler with a period of 5ms.
	def run(self):

		while True:

			if self.state == 0:

				print("MOTOR TASK " + str(self.chiralIndex) + ": Initializing...")
				self.motor.enable()
				self.encoder.update()
				self.prevPower = 0
				self.controlIndex = 0
				self.prevPosition = self.targetPosition.get()
				self.prevVelocity = self.targetVelocity.get()
				self.prevPower = -1
				self.PID = PID(Motor_Task.VEL_P, Motor_Task.VEL_I, Motor_Task.VEL_D, 0, Motor_Task.DER_AVE)
				self.logIndex = 0

				self.state = 1

			elif self.state == 1:

				self.encoder.update()
				self.encoderPosition.put(self.encoder.get_position())
				velocity = self.encoder.get_velocity()

				if self.targetPosition.get() != self.prevPosition:
					self.controlIndex = 1
					self.prevPosition = self.targetPosition.get()


				if self.targetVelocity.get() != self.prevVelocity:
					self.controlIndex = 0
					self.prevVelocity = self.targetVelocity.get()

				nextPower = 0
				if self.controlIndex == 1: # position control
					error = (self.targetPosition.get() - self.encoderPosition.get())
					if abs(error) < Motor_Task.POS_DB:
						nextPower = 0
					else:
						nextPower = error * Motor_Task.POS_P
						if nextPower > Motor_Task.POS_MAX:
							nextPower = Motor_Task.POS_MAX
						elif nextPower < -1*Motor_Task.POS_MAX:
							nextPower = -1*Motor_Task.POS_MAX

				else: # velocity control
					error = (self.targetVelocity.get() - velocity)
					nextPID = self.PID.calculate(error)
					nextFF = velocity*Motor_Task.VEL_FF
					nextPower = int(nextPID + nextFF)

				if nextPower > Motor_Task.MAX_EFFORT:
					nextPower = Motor_Task.MAX_EFFORT
				elif nextPower < -1*Motor_Task.MAX_EFFORT:
					nextPower = -1*Motor_Task.MAX_EFFORT
				if nextPower != self.prevPower: # only set the power when it changes
					self.motor.set_effort(nextPower)
					self.prevPower = nextPower
					
				# # log the current values for pid tuning
				# if self.logIndex > 50:
				# 	print("MOTOR TASK: Desired velocity: " + str(self.targetVelocity.get()) + ", Actual velocity: " + str(velocity) + ", Power: " + str(nextPower))
				# 	self.logIndex = 0
				# else:
				# 	self.logIndex += 1

			else:
				self.state = 0

			yield self.state

	## @var chiralIndex
    #  Index for differentiating between left and right.  0 = left, 1 = right

    ## @var motor
    #  Motor object for setting motor parameters

    ## @var encoder
    #  Encoder object for reading encoder data

    ## @var state
    #  Next state to be run by the state machine.  0 = initialization, 1 = enabled

    ## @var prevPower
    #  The last power assigned to the motor, to avoid assigning the same value again

    ## @var controlIndex
    #  Flag for differentiating between position and velocity control.  0 = position control, 1 = velocity control

    ## @var prevPosition
    #  Last target position, for detecting changes in the target

    ## @var prevVelocity
    #  Last target velocity, for detecting changes in the target

    ## @var PID
    #  PID object for velocity control

    ## @var logIndex
    #  Index value for keeping track of logging data