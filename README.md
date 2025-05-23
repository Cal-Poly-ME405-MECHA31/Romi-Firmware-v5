# ME405 MECHA31 TERM PROJECT # {#mainpage}

## Hello! Welcome to our Mechatronics Term Project.  Created by Evan Long and Sydney Alexander.

The following content documents our completion of Cal Poly's ME405 Mechatronics Term Project.  The overall goal of the
 project was to wire and program a small, two-wheeled robot named Romi to complete an obstacle course set by the 
 instructor.  Over the course of the last several months, we have wired up the machine, installed IR, ultrasonic, and 
 IMU sensors, and programmed it with python firmware capable of reliably completing the obstacle course.

 A video of our robot completing the course is included below:

\htmlonly
<iframe width="560" height="315" src="https://www.youtube.com/embed/644qalbS5Qk?si=99lT6oUaL8Q2IfnB" title="YouTube video player" allowfullscreen></iframe>
\endhtmlonly

We have two goals with this page.  First, we wish to provide a high-level overview of the code structure, with quick 
 navigation to anywhere in the project.  Second, here we will include some of the notable features of our software.

### Links to Each 11 Classes:

1. Main.py: [class](https://cal-poly-me405-mecha31.github.io/class_m_e405_01_term_01_project_01v5_1_1_main.html) | [source](https://cal-poly-me405-mecha31.github.io/m_e405_01_term_01_project_01v5_8py_source.html)
2. Com Task: [class](https://cal-poly-me405-mecha31.github.io/class_com___task_1_1_com___task.html) | [source](https://cal-poly-me405-mecha31.github.io/com___task_8py_source.html)
3. Controller: [class](https://cal-poly-me405-mecha31.github.io/class_controller_1_1_controller.html) | [source](https://cal-poly-me405-mecha31.github.io/controller_8py_source.html)
4. PID: [class](https://cal-poly-me405-mecha31.github.io/class_p_i_d_1_1_p_i_d.html) | [source](https://cal-poly-me405-mecha31.github.io/p_i_d_8py_source.html)
5. Motor: [class](https://cal-poly-me405-mecha31.github.io/class_motor_1_1_motor.html) | [source](https://cal-poly-me405-mecha31.github.io/motor_8py_source.html)
6. Encoder: [class](https://cal-poly-me405-mecha31.github.io/class_encoder_1_1_encoder.html) | [source](https://cal-poly-me405-mecha31.github.io/encoder_8py_source.html)
7. Motor Controller: [class](https://cal-poly-me405-mecha31.github.io/class_motor___task_1_1_motor___task.html) | [source](https://cal-poly-me405-mecha31.github.io/motor___task_8py_source.html)
8. IR Driver: [class](https://cal-poly-me405-mecha31.github.io/class_i_r___sense___task_1_1_i_r___sense___task.html) | [source](https://cal-poly-me405-mecha31.github.io/i_r___sense___task_8py_source.html)
9. IMU Driver: [class](https://cal-poly-me405-mecha31.github.io/class_i_m_u___tracker_1_1_i_m_u___tracker.html) | [source](https://cal-poly-me405-mecha31.github.io/i_m_u___tracker_8py_source.html)
10. Ultrasonic Driver: [class](https://cal-poly-me405-mecha31.github.io/class_ultra___sense___task_1_1_ultra___sense___task.html) | [source](https://cal-poly-me405-mecha31.github.io/ultra___sense___task_8py_source.html)
11. XY Integrator: [class](https://cal-poly-me405-mecha31.github.io/class_x_y___tracking_1_1_x_y___tracking.html) | [source](https://cal-poly-me405-mecha31.github.io/x_y___tracking_8py_source.html)

### Class Descriptions for Context:

1. main.py - Our main file has three goals.  1: Create instances of all share/queue objects.  2. Create instances of 
   all task objects.  3. Create and run the scheduler

2. Com Task - Handles the interaction between the bluetooth module and the other classes.  Com Task handles both
   incoming commands as well as outgoing data transfers

3. Controller - Defines the main state space for navigating the course.  The controller class defines the logic of how
   Romi will respond to sensor inputs from the course.  Controller class also handles navigation error processing

4. PID - A generic, abstracted PID class that is used in various places across the project.  Some key features include
   integral saturation and derivative averaging

5. Motor - A driver for Romi's DC motors.  This class simply provides wrappers around the GPIO interface, but does not
   define any controller behavior of the motor

6. Encoder - A driver for Romi's encoders.  This class does not define the timing of the encoder updates, but rather
   provides a simple interface to update the encoder

7. Motor Controller - In the motor controller task, we define a position and velocity PID controller for Romi's motors,
   using encoder feedback

8. IR Driver - A driver for the IR sensor installed on Romi.  The IR sensors are the RC models, so our IR driver
   defines a list of callbacks for the event when the IR sensor is triggered.  The main outputs are the centroid of
   the line and the strength of the reflectance.

9. IMU Driver - A driver for the IMU installed on Romi.  The IMU automatically defines Romi's Euler angles, so the 
   driver simply waits for calibration success, then reads the heading.

10.  Ultrasonic Diver - A driver for the ultrasonic sensors installed on Romi.  The ultrasonic sensors are the HC-SR04 
     model, which means they behave in a very similar way to the IR sensors.

11.  XY Integrator - A task to continuously integrate velocity into actual cartesian coordinates.  The XY coordinates
     are used for error handling.

### Wiring Diagram:

<img src="https://cal-poly-me405-mecha31.github.io/Wiring%20Diagram">

### Overall Task Diagram:

<img src="https://cal-poly-me405-mecha31.github.io/Task%20Diagram">

### State Space Diagrams:

#### Top Level Control:

<img src="https://cal-poly-me405-mecha31.github.io/Top%20Level%20Diagram">

#### Course Controller:

<img src="https://cal-poly-me405-mecha31.github.io/Controller%20Diagram">

#### Sensor Driver (Abstract):

<img src="https://cal-poly-me405-mecha31.github.io/Sensor%20Diagram">
