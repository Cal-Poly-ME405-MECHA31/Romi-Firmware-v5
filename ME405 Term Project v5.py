import micropython
from pyb import Pin, Timer, UART, ExtInt, I2C
from time import ticks_us, ticks_diff
from array import array
from gc import collect
import struct
import task_share
import cotask

from PID import PID
from Motor import Motor
from Encoder import Encoder
from Motor_Task import Motor_Task
from IR_Sense_Task import IR_Sense_Task
from IMU_Tracker import IMU_Tracker
from Ultra_Sense_Task import Ultra_Sense_Task
from Controller import Controller
from Com_Task import Com_Task
from XY_Tracking import XY_Tracking


## This is the code executed by the MCU on reset.  The main loop that calls the scheduler is included here.
#  While no program logic is included at this level of code, the task object creation does determine the 
#  priority and period of each task.
class Main:

	def __init__(self):
		self.leftMotor = Motor(8, 3, Pin.cpu.C8, Pin.cpu.C6, Pin.cpu.C5)
		self.rightMotor = Motor(2, 2, Pin.cpu.B3, Pin.cpu.B4, Pin.cpu.B5)

		self.leftEncoder = Encoder(1, Pin.cpu.A8, Pin.cpu.A9)
		self.rightEncoder = Encoder(3, Pin.cpu.A6, Pin.cpu.A7)
		
		self.encoderPosition = []
		self.encoderPosition.append(task_share.Share('i'))
		self.encoderPosition.append(task_share.Share('i'))
		self.encoderPosition[0].put(0)
		self.encoderPosition[1].put(0)

		self.dataQueue = task_share.Queue('f', 100) # dataQueue contains a list of data to send over bluetooth

		self.ctrlState = task_share.Share('I') # ctrlState indicates the overall state of the system
		self.ctrlState.put(0) # 0 is idle mode

		self.offset = task_share.Share('i') # current offset of the line as seen by IR sensor
		self.offset.put(0)
		self.strength = task_share.Share('I') # avereage reflectance of the IR sensors
		self.strength.put(0)

		self.targetPosition = []
		self.targetPosition.append(task_share.Share('i'))
		self.targetPosition.append(task_share.Share('i'))
		self.targetPosition[0].put(0)
		self.targetPosition[1].put(0)

		self.targetVelocity = []
		self.targetVelocity.append(task_share.Share('i'))
		self.targetVelocity.append(task_share.Share('i'))
		self.targetVelocity[0].put(0)
		self.targetVelocity[1].put(0)

		self.lineSpeed = task_share.Share('I') # current desired speed along line, ticks/s

		self.kPShare = task_share.Share('f') # current desired line tracking K_P

		self.kIShare = task_share.Share('f') # current desired line tracking K_I

		self.kDShare = task_share.Share('f') # current desired line tracking K_D

		self.heading = task_share.Share('f') # current measured heading
		self.heading.put(0)

		self.calibrated = task_share.Share('I')
		self.calibrated.put(0)

		self.distance = []
		self.distance.append(task_share.Share('i')) # current measured heading
		self.distance.append(task_share.Share('i'))
		self.distance[0].put(0)
		self.distance[1].put(0)

		self.xPosition = task_share.Share('f')
		self.xPosition.put(0.0)
		self.yPosition = task_share.Share('f')
		self.yPosition.put(0.0)

		collect()

	## the main function creates objects for each task and runs the scheduler
	def main(self):

		leftMotorObj = Motor_Task(0, self.leftMotor, self.leftEncoder, self.encoderPosition, self.targetPosition, self.targetVelocity)
		leftMotorTask = cotask.Task(leftMotorObj.run, name = 'Left Motor Task', priority = 2,  period = 5, profile = True, trace = False)
		cotask.task_list.append(leftMotorTask)

		rightMotorObj = Motor_Task(1, self.rightMotor, self.rightEncoder, self.encoderPosition, self.targetPosition, self.targetVelocity)
		rightMotorTask = cotask.Task(rightMotorObj.run, name = 'Right Motor Task', priority = 2,  period = 5, profile = True, trace = False)
		cotask.task_list.append(rightMotorTask)

		IRObj = IR_Sense_Task(self.dataQueue, self.ctrlState, self.offset, self.strength)
		IRTask = cotask.Task(IRObj.run, name = 'IR Sense Task', priority = 0,  period = 2, profile = True, trace = False)
		cotask.task_list.append(IRTask)

		imuObj = IMU_Tracker(self.dataQueue, self.ctrlState, self.targetVelocity, self.heading, self.calibrated)
		imuTask = cotask.Task(imuObj.run, name = 'IMU Tracker Task', priority = 0, period = 20, profile = True, trace = False)
		cotask.task_list.append(imuTask)

		ultraObj = Ultra_Sense_Task(self.dataQueue, self.ctrlState, self.distance)
		ultraTask = cotask.Task(ultraObj.run, name = 'Ultra Sense Task', priority = 0,  period = 10, profile = True, trace = False)
		cotask.task_list.append(ultraTask)

		xyObj = XY_Tracking(self.encoderPosition, self.dataQueue, self.ctrlState, self.heading, self.calibrated, self.xPosition, self.yPosition)
		xyTask = cotask.Task(xyObj.run, name = 'XY Tracking Task', priority = 0,  period = 20, profile = True, trace = False)
		cotask.task_list.append(xyTask)

		controlObj = Controller(self.encoderPosition, self.dataQueue, self.ctrlState, self.offset, self.strength, self.targetVelocity, self.lineSpeed, self.kPShare, self.kIShare, self.kDShare, self.heading, self.calibrated, self.distance, self.xPosition, self.yPosition)
		controlTask = cotask.Task(controlObj.run, name = 'Motion Control Task', priority = 0, period = 10, profile = True, trace = False)
		cotask.task_list.append(controlTask)

		comObj = Com_Task(self.dataQueue, self.ctrlState, self.targetPosition, self.targetVelocity, self.lineSpeed, self.kPShare, self.kIShare, self.kDShare)
		comTask = cotask.Task(comObj.run, name = 'Communication Task', priority = 1,  period = 10, profile = True, trace = False)
		cotask.task_list.append(comTask)

		collect()

		print("MAIN: Entering main task loop: ")

		while True:
			try:

				cotask.task_list.pri_sched()

			except KeyboardInterrupt:
				print("Main: Exiting...\n")

				self.leftMotor.disable()
				self.rightMotor.disable()
				break

			except:

				self.leftMotor.disable()
				self.rightMotor.disable()

				raise

		print('\n' + str (cotask.task_list))
		print(task_share.show_all())
		print(task1.get_trace())
		print('')


	## @var leftMotor
    #  Motor object for setting left motor parameters

    ## @var rightMotor
    #  Motor object for setting right motor parameters

    ## @var leftEncoder
    #  Encoder object for reading left encoder data

    ## @var rightEncoder
    #  Encoder object for reading right encoder data

    ## @var encoderPosition
    #  List containing two Share objects for current encoder position.  0 = left, 1 = right

    ## @var dataQueue
    #  Queue of floats that any task can place values into to send over bluetooth

    ## @var ctrlState
    #  The overall state of the program:
    #  0 - idle state
    #  1 - wasd control mode
    #  2 - IR sense mode
    #  3 - IR calibration mode
    #  4 - Course follow mode
    #  5 - North tracking mode
    #  6 - Ultrasonic sense mode

    ## @var offset
    #  Share containing the current offset of the centroid as read by the IR sensors, in sensor indexes scaled by 100

    ## @var strength
    #  Scaled measure of total strength of reflectance detected by the sensors

    ## @var targetPosition
    #  List containing two Share objects for current target motor position, in ticks.  0 = left, 1 = right

    ## @var targetVelocity
    #  List containing two Share objects for current target motor velocity, in ticks/s.  0 = left, 1 = right

    ## @var lineSpeed
    #  Share object allows user to adjust current lineSpeed as Romi navigates the course

    ## @var kPShare
    #  Share object allows user to adjust current K_P as Romi navigates the course

    ## @var kIShare
    #  Share object allows user to adjust current K_I as Romi navigates the course

    ## @var kDShare
    #  Share object allows user to adjust current K_D as Romi navigates the course

    ## @var heading
    #  Share object contains the most recent heading data as read from the IMU

    ## @var calibrated
    #  Share object contains a boolean indicating if the IMU has acheived calibration or not

    ## @var distance
    #  List containing two Share objects for the current distance measured by the ultrasonic sensors.  0 = forward, 1 = side

    ## @var xPosition
    #  Share object contains the current x position on the course from integrated encoder data

    ## @var yPosition
    #  Share object contains the current y position on the course from integrated encoder data



if __name__ == "__main__":
	print("ME405 MECHA31 Lab 3 Romi Script.  Evan Long and Sydney Alexander")
	print("Connect via bluetooth to 'mecha31' network to control Romi")
	print("Press \'h\' for a list of valid commands")

	micropython.alloc_emergency_exception_buf(100)

	main = Main()
	main.main()