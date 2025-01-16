from servos_sim import *
from camera_sim import *
from pid_control_sim import PID
import logging

logger = logging.getLogger(__name__)
'''
 -> Install Miniconda, create robotsim environment using requirements.txt: (in the terminal, from the repo's directory) conda create -n robotsim --file requirements.txt, activate it: conda activate robotsim
 -> Follow steps 1. to 6. in the Instructions Sheet and then replace "print" with "logger.info" and "time.sleep_ms" to "self.servo.sleep_ms_simulated"
 -> Can edit camera parameters in camera_sim.py
 -> Can add detectable objects subclasses (other than grid cells, obstacles, star) in environment.py
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
        ## 1. different to robot.py: passes self, env to servo,camera, pid; added state to Robot class
        #  Make sure you don't rewrite when pasting code from robot.py
        self.servo = Servo(self)
        self.servo.soft_reset() # TBD create simulation version 
        self.cam = Cam(self,env) 
        self.PID = PID(self,p, i, d, imax)
        self.state = State(self.cam.env.starting_pos[0] * self.cam.env.square_size + self.cam.env.square_size / 2, self.cam.env.starting_pos[1] * self.cam.env.square_size + self.cam.env.square_size / 2,self.cam.env.starting_pos[2],self.cam.env.starting_pos[3])
        
        ## 2.  Blob IDs - can add more or change ids depending on the order of thresholds in the thresholds list
        self.blue_id = 0
        self.green_id = 1
        # self.red_id = 2 # example
        # self.star_id = 3 # example


        ## 3. Add other attributes from robot.py
        # self.scan_direction = 1 # example


    def RoboNav(self, speed: float, bias: float):
        """
        controls the main logic
        """
        logger.info("Starting RoboNav navigation")
        ## 4. Add the code from the function you call to solve the task

        logger.info("Navigation completed")
    

    ## 5. Add any helping functions you used in RoboNav



    ## 6. Edit/ replace the given functions



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


    def scan_for_blob(self, step = 2, limit = 20) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees

        Return:
            pan_angle_to_best_blob - angle of where a blob is found, green or best blue, OR None, if dead end
            is_best_blob_green - is the blob
        """
        counter = 0 #for frame buffer
        is_best_blob_green = False
        best_blue_blob_for_scan = None #the best blob that is blue that was found duirng the whole pan scan
        pan_angle_to_best_blob = None
#        print ("searching")
        pixels = 0
        pan_angle_to_best_blob = 0
        while counter<=1: # Perform one scanning (left and right)
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)
            # Set new angle
            self.servo.set_angle(new_pan_angle)
#            time.sleep_ms(200)
#            print ("searching")
            # Check for green

            green_blobs, _ = self.cam.get_blobs_bottom(self.green_id) #green
            biggest_green_blob_in_frame = self.cam.get_biggest_blob(green_blobs)
            blobs_red, img_2 = self.cam.get_blobs_bottom(self.red_id)

            if (biggest_green_blob_in_frame is not None): #green
                    is_best_blob_green = True
                    pan_angle_to_best_blob = self.servo.pan_pos
#                    counter = 2 # Exit loop
                    break

            # check for blue
            blue_blobs, _ = self.cam.get_blobs_bottom(self.blue_id) #blue
            biggest_blue_blob_in_frame = self.cam.get_biggest_blob(blue_blobs)
            blobs_red, img_2 = self.cam.get_blobs_bottom(self.red_id)
            
            if biggest_blue_blob_in_frame is not None: #blue
#                print("found blue")
                # check if current blue is the best blue
                new_pixels =  biggest_blue_blob_in_frame.pixels()
                if new_pixels > pixels:
                    pixels = new_pixels
                    pan_angle_to_best_blob = new_pan_angle
                # best_blue_blob_for_scan, pan_angle_to_best_blob = self.is_new_blob_better_than_old(
                #         biggest_blue_blob_in_frame,
                #         best_blue_blob_for_scan,
                #         new_pan_angle,
                #         pan_angle_to_best_blob)
            star_blobs, img = self.cam.get_blobs_bottom(self.star_id)
            biggest_star = self.cam.get_biggest_blob(star_blobs)
            if biggest_star is not None:
                logger.debug("Star found in scanning")
                if new_pan_angle>=0:
                    self.star_on_right = -1
                else:
                    self.star_on_right = 1

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                counter += 1
                logger.debug("Not found ...")
                self.scan_direction *= -1
#        print (pan_angle_to_best_blob)
        return pan_angle_to_best_blob, is_best_blob_green




    def debug(self, threshold_idx: int) -> None:
        """
        A debug function for the Robots vision.
        If no block ID is specified, all blocks are detected and debugged.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
        """
        while True:
            blobs, img = self.cam.get_blobs()
            if threshold_idx is not None:
                found_idx = self.cam.find_blob(blobs, threshold_idx)
            else:
                found_idx = range(len(blobs))

            if found_idx:
                for blob in [blobs[i] for i in found_idx]:
                    img.draw_rectangle(blob.rect())
                    img.draw_string(blob.cx(),blob.cy(),str(blob.code()))

                    angle_err = blob.cx() - self.cam.w_centre

                    print('\n' * 2)
                    print('Code:       ', blob.code())
                    print('X-pos:      ',blob.cx())
                    print('Pan angle:  ', self.state.get_gaze())
                    print('Angle err:  ', angle_err)
                    print('Angle corr: ', (angle_err-self.state.get_gaze())/self.servo.max_deg)
                    print('Block size: ', blob.pixels())

                    time.sleep(1)


   