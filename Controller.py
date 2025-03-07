from time import ticks_us, ticks_diff
from pyb import Pin, Timer, UART, ExtInt, I2C
from array import array
import task_share
import cotask
import struct
import math

from PID import PID
from Motor import Motor
from Encoder import Encoder
from Motor_Task import Motor_Task
from IR_Sense_Task import IR_Sense_Task
from IMU_Tracker import IMU_Tracker
from Ultra_Sense_Task import Ultra_Sense_Task

## The Controller class contains the state machine that tracks Romi's progress through the course.
#  The course is divided into 21 individual steps, each expressed as a state that the controller
#  object must pass through before reaching the end.  Each state defines the control law for the motors
#  as well as the logic for passing to the next state.
class Controller:

	## Maximum speed contribution by the line follow integral term before the integral stops increasing
	INTEGRAL_SPEED = 10 
	## Number of previous line forllow derivative speed terms to average when calculating next derivative
	DER_AVE = 3

	INIT = 0
	WAIT = 1
	LINE_1 = 2
	ALIGN_DIAMOND = 3
	DIAMOND = 4
	CURVE = 5
	ALIGN_DOTTED = 6
	DOTTED = 7
	LINE_3 = 8
	OFFSET = 9
	ALIGN_PILLAR = 10
	PILLARS = 11
	ALIGN_LINE_4 = 12
	LINE_4 = 13
	ALIGN_WALL = 14
	ALONG_WALL = 15
	ALONG_WALL_2 = 16
	ALIGN_WALL_2 = 17
	PAST_WALL = 18
	PAST_WALL_2 = 19
	ALIGN_LINE_5 = 20
	LINE_5 = 21
	ALIGN_END = 22
	TO_END = 23
	END = 24

	ALIGN_POINT = 25
	DRIVE_POINT = 26


	def __init__(self, encoderPosition, dataQueue, ctrlState, offset, strength, targetVelocity, lineSpeed, kPShare, kIShare, kDShare, heading, calibrated, distance, xPosition, yPosition):
		self.encoderPosition = []
		self.encoderPosition.append(encoderPosition[0])
		self.encoderPosition.append(encoderPosition[1])

		self.dataQueue = dataQueue
		self.ctrlState = ctrlState

		self.offset = offset
		self.strength = strength

		self.lineSpeed = lineSpeed

		self.targetVelocity = []
		self.targetVelocity.append(targetVelocity[0])
		self.targetVelocity.append(targetVelocity[1])

		self.kPShare = kPShare
		self.kIShare = kIShare
		self.kDShare = kDShare

		self.heading = heading

		self.calibrated = calibrated

		self.distance = []
		self.distance.append(distance[0])
		self.distance.append(distance[1])

		self.xPosition = xPosition
		self.yPosition = yPosition

		self.state = Controller.INIT

	def log(self, val): # function to cut down on redundant code
		self.dataQueue.put(-0.1)
		self.dataQueue.put(-0.1)
		self.dataQueue.put(1)
		self.dataQueue.put(val)


	## State machine generator function that is called by the scheduler.  Each call to this function executes, at most, one state.
	#  Generally, the states either advance to the next state or return to the idle state
	def run(self):

		while True:

			if self.state == Controller.INIT: # initialize

				print("LINE TASK: Initializing...")

				self.linePID = PID(0, 0, 0, Controller.INTEGRAL_SPEED, Controller.DER_AVE) # PID object used track a line
				self.headingPID = PID(IMU_Tracker.K_P, IMU_Tracker.K_I, 0, IMU_Tracker.INTEGRAL_SPEED, IMU_Tracker.DER_AVE) # PID object used to align to a heading
				self.state = Controller.WAIT
				self.returnState = self.state

			elif self.state == Controller.WAIT: # wait for ctrlState to be set to 4

				self.returnState = self.state

				if self.ctrlState.get() == 4:

					if self.calibrated.get() == 0:
						self.log(-1)

						self.ctrlState.put(2)

					else:

						self.linePID.zero()
						self.linePID.setTimeStamp()

						self.startHeading = self.heading.get() # record the start heading for reference later
						self.startPosition = self.encoderPosition[0].get()

						self.kPShare.put(4)
						self.kIShare.put(0)
						self.kDShare.put(0)
						self.lineSpeed.put(1000)

						self.state = Controller.LINE_1
						self.log(self.state)

			elif self.state == Controller.LINE_1: # follow the line until the diamond

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				if (self.encoderPosition[0].get() - self.startPosition > 7000) or ((self.strength.get() > 35) and (self.encoderPosition[0].get() - self.startPosition > 6000)): # follow the line until strength rises above the limit
					self.targetVelocity[0].put(0)
					self.targetVelocity[1].put(0)

					self.headingPID.zero()
					self.headingPID.setTimeStamp()

					self.state = Controller.ALIGN_DIAMOND
					self.log(self.state)

				else:

					if self.strength.get() > 2:

						self.linePID.setPID(self.kPShare.get(), self.kIShare.get(), self.kDShare.get()) # update the constants with user preferences
						yawSpeed = self.linePID.calculate(self.offset.get())

						self.targetVelocity[0].put(self.lineSpeed.get() - yawSpeed) # apply the speeds
						self.targetVelocity[1].put(self.lineSpeed.get() + yawSpeed)

					else:

						self.targetVelocity[0].put(-500) # apply the speeds
						self.targetVelocity[1].put(-500)

			elif self.state == Controller.ALIGN_DIAMOND: # align Romi with the diamond

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				targetHeading = self.startHeading + 90
				if targetHeading > 360:
					targetHeading -= 360

				error = -1*(targetHeading - self.heading.get())
				if error < -180:
					error += 360
				elif error > 180:
					error -= 360

				if abs(error) < 3: # if heading is within tolerance, move to next state
					self.linePID.zero()
					self.linePID.setTimeStamp()

					self.targetVelocity[0].put(0)
					self.targetVelocity[1].put(0)

					self.timeStamp = ticks_us()

					self.state = Controller.DIAMOND
					self.log(self.state)

				else:
					yawSpeed = self.headingPID.calculate(error)
					# self.log(error)

					self.targetVelocity[0].put(-1*yawSpeed)
					self.targetVelocity[1].put(yawSpeed)

			elif self.state == Controller.DIAMOND: # driving past the diamond

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				if (self.strength.get() > 5) and (self.strength.get() < 30) and (ticks_diff(ticks_us(), self.timeStamp) > 1000000):

					self.targetVelocity[0].put(0)
					self.targetVelocity[1].put(0)

					self.linePID.zero()
					self.linePID.setTimeStamp()

					self.kPShare.put(5)
					self.kIShare.put(0)
					self.kDShare.put(0)
					self.lineSpeed.put(500)

					self.state = Controller.CURVE
					self.log(self.state)

				if ticks_diff(ticks_us(), self.timeStamp) < 3000000:

					self.targetVelocity[0].put(1000)
					self.targetVelocity[1].put(1000)

				else:

					self.targetVelocity[0].put(-500)
					self.targetVelocity[1].put(-500)

			elif self.state == Controller.CURVE: # track the line until the dotted lines

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				targetHeading = self.startHeading - 90
				if targetHeading < 360:
					targetHeading += 360

				error = -1*(targetHeading - self.heading.get())
				if error < -180:
					error += 360
				elif error > 180:
					error -= 360

				if (self.strength.get() < 10) or abs(error) < 20:

					self.headingPID.zero()
					self.headingPID.setTimeStamp()

					self.state = Controller.ALIGN_DOTTED
					self.log(self.state)

				else:

					self.linePID.setPID(self.kPShare.get(), self.kIShare.get(), self.kDShare.get())

					yawSpeed = self.linePID.calculate(self.offset.get())

					self.targetVelocity[0].put(self.lineSpeed.get() - yawSpeed)
					self.targetVelocity[1].put(self.lineSpeed.get() + yawSpeed)

			elif self.state == Controller.ALIGN_DOTTED: # align Romi with the dotted lines

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				targetHeading = self.startHeading - 85
				if targetHeading < 360:
					targetHeading += 360

				error = -1*(targetHeading - self.heading.get())
				if error < -180:
					error += 360
				elif error > 180:
					error -= 360

				if abs(error) < 5:
					self.linePID.zero()
					self.linePID.setTimeStamp()

					self.timeStamp = ticks_us()

					self.state = Controller.DOTTED
					self.log(self.state)

				else:
					yawSpeed = self.headingPID.calculate(error)

					self.targetVelocity[0].put(-1*yawSpeed)
					self.targetVelocity[1].put(yawSpeed)

			elif self.state == Controller.DOTTED: # drive past the dotted lines

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				if self.strength.get() > 15:

					self.linePID.zero()
					self.linePID.setTimeStamp()

					self.kPShare.put(4)
					self.kIShare.put(0)
					self.kDShare.put(0)
					self.lineSpeed.put(800)

					self.startPosition = self.encoderPosition[0].get()

					self.headingPID.zero()
					self.headingPID.setTimeStamp()

					self.state = Controller.LINE_3
					self.log(self.state)

				elif ticks_diff(ticks_us(), self.timeStamp) > 3000000:

					self.targetVelocity[0].put(-500)
					self.targetVelocity[1].put(-500)

				else:

					self.targetVelocity[0].put(1000)
					self.targetVelocity[1].put(1000)

			elif self.state == Controller.LINE_3: # follow the line until the pillars

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				if (self.encoderPosition[0].get() - self.startPosition) < 10000:

					if self.strength.get() < 5:

						self.targetVelocity[0].put(-500)
						self.targetVelocity[1].put(-500)

					else:

						self.linePID.setPID(self.kPShare.get(), self.kIShare.get(), self.kDShare.get())

						yawSpeed = self.linePID.calculate(self.offset.get())

						self.targetVelocity[0].put(self.lineSpeed.get() - yawSpeed)
						self.targetVelocity[1].put(self.lineSpeed.get() + yawSpeed)

				elif (self.encoderPosition[0].get() - self.startPosition) < 12500:

					if self.strength.get() < 10:

						self.startPosition = self.encoderPosition[0].get()

						self.state = Controller.OFFSET
						self.log(self.state)

					else:

						if self.strength.get() < 30:

							self.linePID.setPID(self.kPShare.get(), self.kIShare.get(), self.kDShare.get())

							yawSpeed = self.linePID.calculate(self.offset.get())

							self.targetVelocity[0].put(800 - yawSpeed)
							self.targetVelocity[1].put(800 + yawSpeed)

						else:

							targetHeading = self.startHeading - 180
							if targetHeading < 360:
								targetHeading += 360

							error = -1*(targetHeading - self.heading.get())
							if error < -180:
								error += 360
							elif error > 180:
								error -= 360

							yawSpeed = self.headingPID.calculate(error)

							self.targetVelocity[0].put(800 - yawSpeed)
							self.targetVelocity[1].put(800 + yawSpeed)

				else:

					if self.strength.get() < 15:

						self.startPosition = self.encoderPosition[0].get()

						self.state = Controller.OFFSET
						self.log(self.state)

					else:

						self.linePID.setPID(self.kPShare.get(), self.kIShare.get(), self.kDShare.get())

						yawSpeed = self.linePID.calculate(self.offset.get())

						self.targetVelocity[0].put(500 - yawSpeed)
						self.targetVelocity[1].put(500 + yawSpeed)

			elif self.state == Controller.OFFSET:

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				if (self.encoderPosition[0].get() - self.startPosition) > 250:

					self.headingPID.zero()
					self.headingPID.setTimeStamp()

					self.state = Controller.ALIGN_PILLAR
					self.log(self.state)

				self.targetVelocity[0].put(500)
				self.targetVelocity[1].put(500)

			elif self.state == Controller.ALIGN_PILLAR: # we've reached the pillars, align IMU

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				targetHeading = self.startHeading - 179
				if targetHeading < 360:
					targetHeading += 360

				error = -1*(targetHeading - self.heading.get())
				if error < -180:
					error += 360
				elif error > 180:
					error -= 360

				if abs(error) < 3:

					self.startPosition = self.encoderPosition[0].get()

					self.state = Controller.PILLARS
					self.log(self.state)

				else:
					yawSpeed = self.headingPID.calculate(error)

					self.targetVelocity[0].put(-1*yawSpeed)
					self.targetVelocity[1].put(yawSpeed)

			elif self.state == Controller.PILLARS: # track the IMU and drive to the end

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT
				
				if (self.encoderPosition[0].get() - self.startPosition) > 4050:
					self.headingPID.zero()
					self.headingPID.setTimeStamp()

					self.state = Controller.ALIGN_LINE_4
					self.log(self.state)

				else:

					targetHeading = self.startHeading - 180
					if targetHeading < 360:
						targetHeading += 360

					error = -1*(targetHeading - self.heading.get())
					if error < -180:
						error += 360
					elif error > 180:
						error -= 360

					yawSpeed = self.headingPID.calculate(error)

					self.targetVelocity[0].put(500 - yawSpeed)
					self.targetVelocity[1].put(500 + yawSpeed)

			elif self.state == Controller.ALIGN_LINE_4: # align with the line

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				targetHeading = self.startHeading - 90
				if targetHeading < 360:
					targetHeading += 360

				error = -1*(targetHeading - self.heading.get())
				if error < -180:
					error += 360
				elif error > 180:
					error -= 360

				if abs(error) < 5:

					self.kPShare.put(3)
					self.kIShare.put(0)
					self.kDShare.put(0)
					self.lineSpeed.put(1000)

					self.timeStamp = ticks_us()

					self.state = Controller.LINE_4
					self.log(self.state)

				else:
					yawSpeed = self.headingPID.calculate(error)

					self.targetVelocity[0].put(-1*yawSpeed)
					self.targetVelocity[1].put(yawSpeed)

			elif self.state == Controller.LINE_4: # follow the line

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				# self.log(self.distance[0].get())

				if (self.distance[0].get() < 3000) and (ticks_diff(ticks_us(), self.timeStamp) > 1000000):

					self.headingPID.zero()
					self.headingPID.setTimeStamp()

					self.state = Controller.ALIGN_WALL
					self.log(self.state)

				else:

					if self.strength.get() < 10:

						self.targetVelocity[0].put(500)
						self.targetVelocity[1].put(500)

					else:

						self.linePID.setPID(self.kPShare.get(), self.kIShare.get(), self.kDShare.get())

						yawSpeed = self.linePID.calculate(self.offset.get())

						self.targetVelocity[0].put(self.lineSpeed.get() - yawSpeed)
						self.targetVelocity[1].put(self.lineSpeed.get() + yawSpeed)

			elif self.state == Controller.ALIGN_WALL: # align with the wall

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				targetHeading = self.startHeading

				error = -1*(targetHeading - self.heading.get())
				if error < -180:
					error += 360
				elif error > 180:
					error -= 360

				if abs(error) < 5:

					self.timeStamp = ticks_us()

					self.state = Controller.ALONG_WALL
					self.log(self.state)

				else:
					yawSpeed = self.headingPID.calculate(error)

					self.targetVelocity[0].put(-1*yawSpeed)
					self.targetVelocity[1].put(yawSpeed)

			elif self.state == Controller.ALONG_WALL: # drive along the wall

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				# self.log(self.distance[1].get())

				if ((self.distance[1].get() > 5000) and (ticks_diff(ticks_us(), self.timeStamp) > 1500000) or (ticks_diff(ticks_us(), self.timeStamp) > 2000000)):

					self.startPosition = self.encoderPosition[0].get()

					self.state = Controller.ALONG_WALL_2
					self.log(self.state)

				else:

					self.targetVelocity[0].put(800)
					self.targetVelocity[1].put(800)

			elif self.state == Controller.ALONG_WALL_2: # drive just past the wall

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				if (self.encoderPosition[0].get() - self.startPosition) > 1000:

					self.headingPID.zero()
					self.headingPID.setTimeStamp()

					self.state = Controller.ALIGN_WALL_2
					self.log(self.state)

				else:

					self.targetVelocity[0].put(800)
					self.targetVelocity[1].put(800)

			elif self.state == Controller.ALIGN_WALL_2: # align with the wall

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				targetHeading = self.startHeading - 90
				if targetHeading < 360:
					targetHeading += 360

				error = -1*(targetHeading - self.heading.get())
				if error < -180:
					error += 360
				elif error > 180:
					error -= 360

				if abs(error) < 5:

					self.state = Controller.PAST_WALL
					self.log(self.state)

				else:
					yawSpeed = self.headingPID.calculate(error)

					self.targetVelocity[0].put(-1*yawSpeed)
					self.targetVelocity[1].put(yawSpeed)

			elif self.state == Controller.PAST_WALL: # drive past the wall

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				# self.log(self.distance[1].get())

				if self.strength.get() > 15:

					self.startPosition = self.encoderPosition[0].get()

					self.state = Controller.PAST_WALL_2
					self.log(self.state)

				else:

					self.targetVelocity[0].put(800)
					self.targetVelocity[1].put(800)

			elif self.state == Controller.PAST_WALL_2:

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				if (self.encoderPosition[0].get() - self.startPosition) > 500:

					self.headingPID.zero()
					self.headingPID.setTimeStamp()

					self.state = Controller.ALIGN_LINE_5
					self.log(self.state)

				else:

					self.targetVelocity[0].put(800)
					self.targetVelocity[1].put(800)


			elif self.state == Controller.ALIGN_LINE_5: # align with the line

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				targetHeading = self.startHeading - 180
				if targetHeading < 360:
					targetHeading += 360

				error = -1*(targetHeading - self.heading.get())
				if error < -180:
					error += 360
				elif error > 180:
					error -= 360

				if abs(error) < 5:

					self.linePID.zero()
					self.linePID.setTimeStamp()

					self.kPShare.put(3)
					self.kIShare.put(0)
					self.kDShare.put(0)
					self.lineSpeed.put(1000)

					self.state = Controller.LINE_5
					self.log(self.state)

				else:
					yawSpeed = self.headingPID.calculate(error)

					self.targetVelocity[0].put(-1*yawSpeed)
					self.targetVelocity[1].put(yawSpeed)

			elif self.state == Controller.LINE_5: # follow the line

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				if self.strength.get() < 10:

					self.headingPID.zero()
					self.headingPID.setTimeStamp()

					self.startPosition = self.encoderPosition[0].get()

					self.state = Controller.TO_END
					self.log(self.state)

				else:

					self.linePID.setPID(self.kPShare.get(), self.kIShare.get(), self.kDShare.get())

					yawSpeed = self.linePID.calculate(self.offset.get())

					self.targetVelocity[0].put(self.lineSpeed.get() - yawSpeed)
					self.targetVelocity[1].put(self.lineSpeed.get() + yawSpeed)


			elif self.state == Controller.TO_END:

				self.returnState = self.state

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				if (self.encoderPosition[0].get() - self.startPosition) > 800:

					self.targetVelocity[0].put(0)
					self.targetVelocity[1].put(0)

					self.state = Controller.END
					self.log(self.state)

					# self.xTarget = -0.5
					# self.yTarget = 0

					# self.returnState = Controller.END

					# self.headingPID.zero()
					# self.headingPID.setTimeStamp()

					# self.log(-1)
					# self.state = Controller.ALIGN_POINT

				else:

					self.targetVelocity[0].put(500)
					self.targetVelocity[1].put(500)

			elif self.state == Controller.END: # end

				self.returnState = self.state

				self.targetVelocity[0].put(0)
				self.targetVelocity[1].put(0)

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

			elif self.state == Controller.ALIGN_POINT:

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				theta = math.atan2(self.yTarget - self.yPosition.get(), self.xPosition.get() - self.xTarget) * 180 / 3.14159

				targetHeading = self.startHeading + theta
				if targetHeading > 360:
					targetHeading -= 360
				elif targetHeading < 360:
					targetHeading += 360

				error = -1*(targetHeading - self.heading.get())
				if error < -180:
					error += 360
				elif error > 180:
					error -= 360

				if abs(error) < 5:

					self.startPosition = self.encoderPosition[0].get()
					self.endPosition = self.startPosition + math.sqrt((self.yTarget - self.yPosition.get())**2 + (self.xPosition.get() - self.xTarget)**2) / 0.035 / (2*3.14159) * 1440

					self.state = Controller.DRIVE_POINT

				else:
					yawSpeed = self.headingPID.calculate(error)

					self.targetVelocity[0].put(-1*yawSpeed)
					self.targetVelocity[1].put(yawSpeed)
				

			elif self.state == Controller.DRIVE_POINT:

				if self.ctrlState.get() != 4:
					self.state = Controller.WAIT

				if self.encoderPosition[0].get() > self.endPosition:

					self.headingPID.zero()
					self.headingPID.setTimeStamp()

					self.state = self.returnState
					self.log(self.state)

				else:

					self.targetVelocity[0].put(500)
					self.targetVelocity[1].put(500)


			else:
				self.state = Controller.INIT

			yield self.state


	## @var state
    #  The next state the machine should run upon being called.  For a list of valid states, see the integer constants from 0-25 defined in this class.

	## @var linePID
    #  PID object with constants chosen for controlling yaw speed as a function of IR sensor offset

    ## @var headingPID
    #  PID object with constants chosen for controlling yaw speed as a function of heading error

    ## @var returnState
    #  In the event that the ALIGN_POINT task is used, returnState defines where the state machine should return to after reaching the point

    ## @var startHeading
    #  This float object is set once when the course is started with the current heading from the IMU, then referenced later for alignment motions

    ## @var startPosition
    #  For movements that require distance tracking of the encoder, this object contains the initial position for later reference

    ## @var timeStamp
    #  For movements that have timeouts, this object containst the initial time stamp for later reference

    ## @var endPosition
    #  Stop position for the DRIVE_POINT state