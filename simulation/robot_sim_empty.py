from servos_sim import *
from camera_sim import *
from pid_control_sim import PID
import logging

logger = logging.getLogger(__name__)
'''
 -> Install Miniconda, create robotsim environment using requirements.txt: (in the terminal, from the repo's directory) conda create -n robotsim --file requirements.txt, activate it: conda activate robotsim
 -> Follow steps 1. to 4. and then replace all "print" with "logger.info" and "time.sleep_ms" with "self.servo.sleep_ms_simulated"
 -> Can edit camera parameters in camera_sim.py
 -> Can add detectable objects subclasses (other than grid cells, obstacles) in environment.py; make sure you add them to the list of detectable objects in the Environment init
 -> Can access the robot_states.csv and snapshots.csv for further analysis
 -> Can add test cases in test_simulation.py
 -> Extra functions defined in camera.py or servos.py will need a simulated version in camera_sim.py or servos_sim.py

'''
class Robot(object):
    """
    A class to manage the functions of a robot for driving and tracking purposes using a camera and servos.
    """
    def __init__(self, env, gain = 7, p=0.15, i=0, d=0, imax=0.01):
        """
        Initializes the Robot object with given PID parameters.

        Args:
            thresholds (list): Colour detection thresholds
            gain (float): Camera gain
            p (float): Proportional gain for the PID.
            i (float): Integral gain for the PID.
            d (float): Derivative gain for the PID.
            imax (float): Maximum Integral error for the PID.
        """
       ## 1. make sure you don't overwrite these --------
        self.servo = Servo(self,env.servo_error,env.pan_error,env.pan_delay,env.speed_coef)
        self.cam = Cam(self,env) # 
        self.PID = PID(self,p, i, d, imax)
        self.WHEEL_BASE = 0.085
        self.state = State(self.cam.env.starting_pos[0] * self.cam.env.square_size + self.cam.env.square_size / 2, self.cam.env.starting_pos[1] * self.cam.env.square_size + self.cam.env.square_size / 2,self.cam.env.starting_pos[2],self.cam.env.starting_pos[3])
        ## -------------------------------------------
        
        ## 2. Copy everything else that you had in the robot.py file
        
        #  Blob IDs - can add more or change ids depending on the order of thresholds in the thresholds list
        self.blue_id = 0
        self.green_id = 1
        self.red_id = 2 # example
        # self.star_id = 3 # example


        # Other attributes from robot.py
        # self.scan_direction = 1 # example

## 3. Make sure the main function you call to solve the task is RoboNav. 
    def RoboNav(self, speed: float, bias: float):
        """
        controls the main logic
        """
        logger.info("Starting RoboNav navigation")
        

        logger.info("Navigation completed")
    

## 4. Add below all the other functions you defined.

#  e.g.
    def drive(self, drive: float, steering: float) -> None:
        self.servo.set_differential_drive(drive, steering)


    def track_blob(self, blob) -> float:
        """
        Adjust the camera pan angle to track a specified blob based on its ID.
        Args
            blob: The blob object to be tracked
        """
        # Error between camera angle and target in pixels
        pixel_error = blob.cx() - self.cam.w_centre

        # Convert error to angle
        angle_error = -(pixel_error/self.cam.w_centre*2*self.cam.h_fov)

        pid_error = self.PID.get_pid(angle_error,1)

        # Error between camera angle and target in ([deg])
        pan_angle = self.servo.pan_pos + pid_error

        # Move pan servo to track block
        self.servo.set_angle(pan_angle)
        logger.debug(f"Tracking metrics - pixel_error: {pixel_error}, angle_error: {angle_error}, pid_error: {pid_error}, pan_angle: {pan_angle}")

        return pan_angle

   