import micropython
from time import ticks_us, ticks_diff
from pyb import Pin, Timer, UART, ExtInt, I2C
from array import array
import task_share
import cotask
import struct

from PID import PID

## The IMU_Tracker class provides a driver to track the heading as calculated by the BNO055 IMU.  The class waits for a succcessful calibration of the IMU,
#  then begins regularly reading the heading over I2C
class IMU_Tracker:

	## Recommended K_P value for PID control of heading
	K_P = 20
	## Recommended K_I value for PID control of heading
	K_I = 2
	## Recommended K_D value for PID control of heading
	K_D = 0
	## Recommended maximum integral speed for PID control of heading
	INTEGRAL_SPEED = 100
	## Recommended number of derivative values to average over time for PID control of heading
	DER_AVE = 10

	def __init__(self, dataQueue, ctrlState, targetVelocity, heading, calibrated):
		self.dataQueue = dataQueue
		self.ctrlState = ctrlState

		self.targetVelocity = []
		self.targetVelocity.append(targetVelocity[0])
		self.targetVelocity.append(targetVelocity[1])

		self.heading = heading

		self.calibrated = calibrated

		self.state = 0

	## run() runs the next state of the state machine.  It is intended to be called by the scheduler with a period of 20ms.
	def run(self):

		while True:

			if self.state == 0:

				print("IMU TRACKER: Initializing...")

				self.imu = I2C(1, I2C.CONTROLLER)
				self.imu.init(I2C.CONTROLLER, baudrate = 400000)
				self.address = self.imu.scan()[0] # IMU is the only device connected
				self.imu.mem_write(12, self.address, 61) # set the IMU into NDOF fusion mode
				self.state = 1

			elif self.state == 1:

				buf = bytearray(0 for n in range(1))
				self.imu.mem_read(buf, self.address, 53) # read the calibration status
				if buf[0] >= 63: # if the calibration status is equal to or greater than 00111111 we are calibrated
					print("IMU TRACKER: Calibration successful")
					self.calibrated.put(1)
					self.state = 2

			elif self.state == 2:

				buf = bytearray(0 for n in range(2))
				self.imu.mem_read(buf, self.address, 26) # read the heading
				self.heading.put(struct.unpack("<h", buf)[0] / 16)

				if self.ctrlState.get() == 5:
					self.PID = PID(IMU_Tracker.K_P, IMU_Tracker.K_I, IMU_Tracker.K_D, IMU_Tracker.INTEGRAL_SPEED, IMU_Tracker.DER_AVE)
					self.state = 3

			elif self.state == 3:

				if self.ctrlState.get() != 5:
					self.state = 2

				buf = bytearray(0 for n in range(2))
				self.imu.mem_read(buf, self.address, 26)
				self.heading.put(struct.unpack("<h", buf)[0] / 16)
				print("IMU TRACKER: Heading: " + str(self.heading.get()))

				error = 0.0
				if self.heading.get() < 180:
					error = self.heading.get()
				else:
					error = self.heading.get() - 360
				yawSpeed = self.PID.calculate(error)

				self.targetVelocity[0].put(-1*yawSpeed)
				self.targetVelocity[1].put(yawSpeed)

			else:

				self.state = 0

			yield self.state

	## @var state
    #  Next state to be run by the state machine.  0 = initialization, 1 = waiting for calibration, 2 = sensing, 3 = north tracking mode

    ## @var imu
    #  I2C object for communication with the IMU

    ## @var address
    #  I2C address of the IMU

    ## @var PID
    #  PID object for controlling yaw speed for North tracking mode