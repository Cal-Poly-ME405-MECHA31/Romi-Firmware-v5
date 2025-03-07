import micropython
from time import ticks_us, ticks_diff
from pyb import Pin, Timer, UART, ExtInt, I2C
from array import array
import task_share
import cotask
import struct

from PID import PID
from Motor import Motor
from Encoder import Encoder
from Motor_Task import Motor_Task
from IR_Sense_Task import IR_Sense_Task
from IMU_Tracker import IMU_Tracker
from Ultra_Sense_Task import Ultra_Sense_Task
from Controller import Controller


## Com_Task handles all interactions with the Bluetooth modele in a single loop.  This happens in three steps.  First,
#  all data is extracted from the UART object and stored in a local buffer.  Second, one command is parsed from the buffer.
#  Third, data is passed from the dataQueue object to the bluetooth module.
class Com_Task:
    ## The minimum period between valid button presses.  This threshold is used to debounce user input
    TOGGLE_PERIOD = 1000000 
    ## Defines the number of encoder ticks to travel forward in wasd mode
    FORWARD_INC = 500
    ## Defines the number of encoder ticks to travel when turning in wasd mode
    TURN_INC = 100 

    ## callback function for user button.  Feature is currently disabled.
    def callback(self, pin):
        if ticks_diff(ticks_us(), self.buttonStamp) > 1000000:
            self.resetCom = True

    def __init__(self, dataQueue, ctrlState, targetPosition, targetVelocity, lineSpeed, kPShare, kIShare, kDShare):
        self.dataQueue = dataQueue

        self.ctrlState = ctrlState

        self.targetPosition = []
        self.targetPosition.append(targetPosition[0])
        self.targetPosition.append(targetPosition[1])

        self.targetVelocity = []
        self.targetVelocity.append(targetVelocity[0])
        self.targetVelocity.append(targetVelocity[1])

        self.lineSpeed = lineSpeed

        self.kPShare = kPShare
        self.kIShare = kIShare
        self.kDShare = kDShare

        self.state = 0

    ## run() defines the code that is run each pass through the state machine.  The class is intended to be run with a scheduler
    #  but will work in a round robin loop as well.
    def run(self):

        while True:

            if self.state == 0:

                print("COM TASK: Initializing...")

                self.serial = UART(1, 115200)  # initialize bluetooth communication
                self.inBuf = ""
                self.outBuf = []
                self.toggleTS = ticks_us()
                self.resetCom = False
                self.buttonStamp = ticks_us()
                # self.buttonReset = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, self.callback)

                self.state = 1

            elif self.state == 1:  # main receive state

                if self.resetCom:  # button has been pressed, reset the COM
                    self.resetCom = False
                    self.serial = UART(1, 115200)

                if self.ctrlState.get() in [0, 2, 3]:  # force motors to 0 in these states
                    self.targetVelocity[0].put(0)
                    self.targetVelocity[1].put(0)

                if self.serial.any() > 0:
                    inputBytes = self.serial.read(self.serial.any())  # read any new bytes

                    inputStr = ""
                    try:
                        inputStr = inputBytes.decode()  # try to decode bytes into str
                    except AttributeError:
                        inputStr = ""

                    self.inBuf += inputStr  # add str to buffer

                if len(self.inBuf) > 0:
                    if self.ctrlState.get() != 4:
                        command = self.inBuf[0]  # grab next command

                        while True:
                            try:
                                self.inBuf = self.inBuf[
                                             1:]  # delete the first character as long as the character is the same as the command

                                if self.inBuf[0] != command:
                                    break
                            except IndexError:
                                break

                        if command == 'h':  # print help message
                            outStr = "ME405 Lab 3 MECHA31 Romi Control Script."
                            outStr += "  Controls are as follows:\r\n"
                            outStr += "    -\'m\': toggle movement mode\r\n"
                            outStr += "    -\'i\': toggle IR sense mode\r\n"
                            outStr += "    -\'r\': set IR reference\r\n"
                            outStr += "    -\'l\': begin line follow\r\n"

                            print(outStr)
                            self.serial.write(str("\r\n" + outStr + "\r\n"))

                        if command == 'm': # put Romi in move mode
                        	if ticks_diff(ticks_us(), self.toggleTS) > Com_Task.TOGGLE_PERIOD:
                        		if self.ctrlState.get() == 1:
                        			self.ctrlState.put(0)
                        			print("COM TASK: Entering idle mode...")
                        		else:
                        			self.ctrlState.put(1)
                        			print("COM TASK: Entering drive mode...")
                        		self.toggleTS = ticks_us()

                        if command == 'i':  # toggle on the IR sensor
                            if ticks_diff(ticks_us(), self.toggleTS) > Com_Task.TOGGLE_PERIOD:
                                if self.ctrlState.get() == 2:
                                    self.ctrlState.put(0)
                                    print("COM TASK: Entering idle mode...")
                                else:
                                    self.ctrlState.put(2)  # ctrlState == 2 turns IR sense on
                                    print("COM TASK: Entering IR sense mode...")
                                self.toggleTS = ticks_us()

                        if command == 'r':  # set the IR references
                            if ticks_diff(ticks_us(), self.toggleTS) > Com_Task.TOGGLE_PERIOD:
                                self.ctrlState.put(3)  # ctrlState == 3 sets IR reference
                                print("COM TASK: Setting reference...")
                                self.toggleTS = ticks_us()

                        if command == 'l':  # toggle line follow mode
                            if ticks_diff(ticks_us(), self.toggleTS) > Com_Task.TOGGLE_PERIOD:
                                if self.ctrlState.get() == 4:
                                    self.ctrlState.put(0)
                                    print("COM TASK: Entering idle mode...")
                                elif self.ctrlState.get() == 2:
                                    self.ctrlState.put(4)  # ctrlState == 4 is line follow mode
                                    print("COM TASK: Entering line follow mode...")
                                self.toggleTS = ticks_us()

                    # # toggle north tracking mode (commented to save RAM)
                    if command == 'n':
                    	if ticks_diff(ticks_us(), self.toggleTS) > Com_Task.TOGGLE_PERIOD:
                    		if self.ctrlState.get() == 5:
                    			self.ctrlState.put(0)
                    			print("COM TASK: Entering idle mode...")
                    		else:
                    			self.ctrlState.put(5) # ctrlState == 5 is north tracking mode
                    			print("COM TASK: Entering north tracking mode...")
                    		self.toggleTS = ticks_us()

                    if command == 'u':
                            if ticks_diff(ticks_us(), self.toggleTS) > Com_Task.TOGGLE_PERIOD:
                                if self.ctrlState.get() == 6:
                                    self.ctrlState.put(0)
                                    print("COM TASK: Entering idle mode...")
                                else:
                                    self.ctrlState.put(6) # ctrlState == 5 is north tracking mode
                                    print("COM TASK: Entering ultra sense mode...")
                                self.toggleTS = ticks_us()

                    # direct movement control (commented to save RAM)
                    if self.ctrlState.get() == 1:
                    	if "w" == command:
                    		self.targetPosition[0].put(self.targetPosition[0].get() + Com_Task.FORWARD_INC)
                    		self.targetPosition[1].put(self.targetPosition[1].get() + Com_Task.FORWARD_INC)
                    	elif "a" == command:
                    		self.targetPosition[0].put(self.targetPosition[0].get() - Com_Task.TURN_INC)
                    		self.targetPosition[1].put(self.targetPosition[1].get() + Com_Task.TURN_INC)
                    	elif "d" == command:
                    		self.targetPosition[0].put(self.targetPosition[0].get() + Com_Task.TURN_INC)
                    		self.targetPosition[1].put(self.targetPosition[1].get() - Com_Task.TURN_INC)
                    	elif "s" == command:
                    		self.targetPosition[0].put(self.targetPosition[0].get() - Com_Task.FORWARD_INC)
                    		self.targetPosition[1].put(self.targetPosition[1].get() - Com_Task.FORWARD_INC)

                    else:  # in the line tracking mode:
                        if 'l' in self.inBuf:  # l indicates we should exit line tracking mode
                            self.ctrlState.put(2)
                            self.inBuf = ""
                        else:
                            for i, char in enumerate(self.inBuf):
                                if char in "pide":  # check if user has set constants
                                    numStr = ""
                                    complete = False
                                    try:
                                        for nextChar in self.inBuf[i + 1:]:
                                            if nextChar == char:
                                                complete = True
                                                break

                                            numStr += nextChar
                                    except IndexError:
                                        pass
                                    if complete:  # only parse complete messages
                                        try:
                                            if char == 'p':
                                                K_P = float(numStr)
                                                self.kPShare.put(K_P)
                                            elif char == 'i':
                                                K_I = float(numStr)
                                                self.kIShare.put(K_I)
                                            elif char == 'd':
                                                K_D = float(numStr)
                                                self.kDShare.put(K_D)
                                            elif char == 'e':
                                                effort = int(numStr)
                                                self.lineSpeed.put(effort)
                                            for nextI, nextChar in enumerate(self.inBuf):
                                                if (nextI > i) and (nextChar == char):
                                                    self.inBuf = self.inBuf[nextI:]

                                        except ValueError:
                                            pass

                if self.dataQueue.any():  # empty the dataQueue into outBuf
                    while True:
                        self.outBuf.append(self.dataQueue.get())
                        if self.dataQueue.any() == False:
                            break

                    self.dataQueue.clear()

                for i, val in enumerate(self.outBuf):
                    if val == -0.1:
                        nextVal = 0
                        try:
                            nextVal = self.outBuf[i + 1]
                        except IndexError:
                            continue

                        if nextVal == -0.1:  # check for to -0.1 values in a row
                            index = self.outBuf[i + 2]
                            if index == 0:
                            	self.serial.write('----------------------------------')
                            	self.serial.write('-------------------------\r\n\r\n')
                            	self.serial.write('CSV START\r\n\r\n')
                            	self.serial.write('----------------------------------')
                            	self.serial.write('-------------------------\r\n\r\n')
                            elif index == -1:
                            	self.serial.write('----------------------------------')
                            	self.serial.write('-------------------------\r\n\r\n')
                            	self.serial.write('CSV END\r\n\r\n')
                            	self.serial.write('----------------------------------')
                            	self.serial.write('-------------------------\r\n\r\n')

                            	break

                            self.outBuf = self.outBuf[i + 3:]

                            outStr = ""
                            index = 0
                            while True:
                                try:
                                    if (self.outBuf[0] == -0.1) and (index > 1):
                                        break
                                    else:
                                        outStr += str(self.outBuf[0])  # add next value to str
                                        outStr += ","
                                        self.outBuf = self.outBuf[1:]  # remove the next value from the buffer
                                        index += 1

                                except IndexError:
                                    break

                            outStr += "\r\n"

                            self.serial.write(outStr.encode("utf-8"))

                            break

            else:
                self.state = 0

            yield self.state

    ## @var resetCOM
    #  A flag that can be set to reinitiailze the UART object for bluetooth communication.  Currently this feature is disabled.

    ## @var state
    #  The next state to be run by the machine upon being called.  0 = initialization, 1 = receiving / transmitting

    ## @var serial
    #  UART object for communicating via the bluetooth module

    ## @var outBuf
    #  A string object that gets dynamically allocated to ensure that all data is extracted from the dataQueue every loop around.  outBuf will never be larger than dataQueue (100 floats)

    ## @var toggleTS
    #  Timestamp of the last user input.  Used with TOGGLE_PERIOD to debounce user input
