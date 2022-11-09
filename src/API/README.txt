robotControl offers a set of functions for... controlling your robot! There are functions for driving and kicking. 

Driving functions only affect the driving motors (ports B and C for right and left motors, respectively). Kicking functions only affect the kicking motor (port A). For example, all_stop() stops the driving motors, but doesn't affect the kicker. Naturally, these functions assume that your robot is set up according to Kevin's specification as posted on https://bitbucket.org/solvents/utsc-robo-soccer/issue/1/robot-port-interface

Each function sets the "throttle" of the respective motors. For example, pivot_left_speed(20) sets the left motor to -20, and the right motor to 20. Timings and angles are left up to the AI to figure out. 

When a function is called, the robot continue to perform that task until another function is called. For example, when drive() is called, the robot will drive forwards at max speed until any other driving function overrides it ... for example all_stop(), which stops the robot. This applies to kicking as well! For example, kick() will drive the kicking motor until stop_kicker() or retract() are called. This was done on purpose because each robot might do different things with their kicker.

Each function (both driving and kicking) typically comes in two variants: basic, and ones ending in "_speed". The basic functions default to performing their task at default speed, while the _speed variants will perform their task at the speed specified by the integer parameter. For example, turn_left_reverse() will keep the right track stationary while driving the left one in reverse at maximum speed. On the other hand, turn_left_reverse_speed(30) will keep the right track stationary while driving the left one in reverse at speed 30. Valid motor speeds go from -100 to 100.

The return values will be positive if the function successfully executed, and negative otherwise.

There are a few macros to pay attention to, all in robotControl.h:
DEFAULT_SPEED is the speed that the basic driving variants run at. By default, it's 100 (maximum speed).
DIRECTION should be set to 1 or -1, depending on the direction that the driving motors are facing.
KICKER_DIRECTION should be set to 1 or -1, depending on the direction that the kicking motor is facing.
POWER_FACTOR_LEFT/RIGHT are there to account for power differences between the left and right motors. They should be set to a number between 0 and 1 (inclusive) in order to scale the power of the given motor down. For example, if we wish to decrease the overall power of the left motor by 20%, we would set POWER_FACTOR_LEFT to 0.8. 
------------------------------------------------------------------------------

Here is a list and description of basic function variants. Unless otherwise stated, all functions have a _speed variant.
//general driving
int drive(); //drive forwards
int drive_speed (int speed); //drive forwards (positive speed) or backwards (negative speed)
int reverse(); //drive backwards at max speed.
int all_stop(); //stop the motors (ignoring the kicker, of course). NO _SPEED VARIANT

//functions for turning left
int pivot_left(); //pivot left on the spot
int turn_left(); //turn left by keeping left track stationary, but advancing right
int turn_left_reverse(); //turn left by keeping right track stationary, but reversing left

//functions for turning right
int pivot_right(); //pivot right on the spot
int turn_right(); //turn right by keeping right track stationary, but advancing left
int turn_right_reverse(); //turn right by keeping left track stationary, but reversing right

//kick
int kick(); //drive kicking arm to deliver a kick
int stop_kicker(); //stop kicking arm. NO _SPEED VARIANT
int retract(); //retract kicking arm

------------------------------------------------------------------------------
Finally, there is one last function:
int drive_custom (int left_speed, int right_speed);

You are NOT intended to use this function as part of typical maneuvers. It is there just because people might want to directly assign custom speeds to both motors.
