from servos_sim import *
from camera_sim import *
from pid_control_sim import PID
import logging

logger = logging.getLogger(__name__)

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
        ##
        self.servo = Servo(self,env.servo_error,env.pan_error)
        # self.servo.soft_reset()
        self.cam = Cam(self,env) # 
        self.PID = PID(self,p, i, d, imax)
        self.WHEEL_BASE = 0.085
        ##
        # Blob IDs - can add more or change ids depending on the order of thresholds in the thresholds list
        self.blue_id = 0
        self.green_id = 1
        self.red_id = 2
        self.star_id = 3

        ###
        self.state = State(self.cam.env.starting_pos[0] * self.cam.env.square_size + self.cam.env.square_size / 2, self.cam.env.starting_pos[1] * self.cam.env.square_size + self.cam.env.square_size / 2,self.cam.env.starting_pos[2],self.cam.env.starting_pos[3])

    # added from real robot.py
        self.star_is_right = 1 # 1 means it's on the left (positive bias means turning left)

        self.scan_direction = 1
        self.camera_height = 6

        self.alpha = self.cam.camera_elevation_angle


    def RoboNav(self, speed: float, bias: float):
        """
        controls the main logic
        """
        logger.info("Starting RoboNav navigation")
        target_has_been_found = False
        target_reached = False

        while target_reached is False:
            blue_found = self.is_blob_in_frame(self.blue_id)
            green_found = self.is_blob_in_frame(self.green_id)

            if target_has_been_found:
                logger.debug(f"Target has been found ")
                if not green_found:
                    logger.info(f"Target passed, finishing navigation ")
                    target_reached = True
                    self.drive(0.2, 0)
                    self.servo.sleep_ms_simulated(700)
                    self.drive(0, 0)
                elif green_found:
                    logger.debug(f"Moving to target ")
                    target_has_been_found = True
                    target_reached = self.go_to_blob(speed, bias, self.green_id)
            
            elif green_found:
                logger.info(f"Target detected ")
                target_has_been_found = True
                target_reached = self.go_to_blob(speed, bias, self.green_id)
            
            elif blue_found:
                logger.debug(f"Following blue path ")
                target_has_been_found = self.go_to_blob(speed, bias, self.blue_id)
            
            else:
                logger.debug("Searching for path")
                self.searching_manager()

        logger.info("Navigation completed")


    def is_blob_in_frame(self, index: int) -> bool:
        """
        Checks if there is a blob if a colour in frame
        RETURN: bool
        """
        logger.debug("Checking if blob is in frame")
        blobs, img = self.cam.get_blobs_bottom(index)
        found_blue = self.cam.get_biggest_blob(blobs)


        # if found_blue is not None:
        #     found_blue = self.is_valid(found_blue)
        return True if found_blue is not None else False

    def wait_for_camera(self) -> None:
        """
        buffer for the camera gain to start working
        """
        logger.debug("Waiting for camera initialization")
        blobs, img = self.cam.get_blobs_bottom(1)
        self.servo.sleep_ms_simulated(1000)

        blobs, img = self.cam.get_blobs_bottom(1)

    def go_to_blob(self, speed: float, bias: float, blob_id: int ) -> None:
        """
        Drive to blob by keeping it in the middle of the frame
        """
        self.servo.set_angle(0)
        counter = 0
        while counter < 1: #frame buffer 2 for actual robot

            if blob_id is not self.green_id:
                logger.debug(f"Looking for green blob ")
                #look for green in every loop, in case green appears while going to blue
                green_blobs, img = self.cam.get_blobs_bottom(self.green_id)
                found_biggest_green = self.cam.get_biggest_blob(green_blobs)

                # if found_biggest_green is not None:
                #     print("found green, check if valid")
                #     found_biggest_green = self.is_valid(found_biggest_green)

                if found_biggest_green is not None:
                    logger.info(f"Green blob detected ")
                    self.drive(0, 0)
                    logger.debug("Switching target blob IDs")
                    return True

            # if no green found, look for the next blob (depending on blob_id)
            logger.debug(f"Going to blob, attempt {counter}")
            blobs, img = self.cam.get_blobs_bottom(blob_id)
            found_biggest = self.cam.get_biggest_blob(blobs)

            # if found_biggest is not None:
            #     found_biggest = self.is_valid(found_biggest)

            if found_biggest is not None:
                logger.debug("Driving to assigned blob")
                counter=0
                logger.debug(f"Blob detected at x={found_biggest.cx()}, id={found_biggest.color_id}")
                logger.debug(f"Camera center at x={self.cam.w_centre}")
                steering = round((self.cam.w_centre-found_biggest.cx())/(2*self.cam.w_centre),3) 
                logger.debug(f"Steering adjustment: {steering}")
                self.drive(speed, steering)
                self.servo.sleep_ms_simulated(50)  
                
               
            else:
                logger.debug(f"Lost assigned blob, attempt {counter}")
                counter+=1

        self.drive(0,0)
        return False

    def searching_manager(self) -> None:
        """
        1: pan to find blobs
           - search for blue and green
           - if green found, choose green
           - else, choose the best blue -> return (return pan angle where blob detected)
                criteria:
                    size
                    direction - prioritize the ones close to the sun/preffered direction
            2: validate blob
                - make sure not behind red
        3: if not valid again, dead_end_manager
        """
        logger.debug("Starting blob search")
        self.drive(0, 0)
        pan_angle_to_best_blob, is_best_blob_green = self.scan_for_blob(step=2, limit=40)
        logger.info(f"Pan angle to best blob: {pan_angle_to_best_blob} ")

        if (is_best_blob_green is True):
            logger.info(f"Target acquired, heading to green ")
            self.servo.set_angle(pan_angle_to_best_blob)
            self.servo.sleep_ms_simulated(200)

            self.turn_to_blob(self.green_id, pan_angle_to_best_blob, self.find_dir_from_angle(pan_angle_to_best_blob))
        elif (pan_angle_to_best_blob is None):
            logger.info("Dead end reached")
            logger.debug("Initiating dead end recovery")
            print("reach deadend")
            print("dead end")
            self.dead_end_exit()
        else:
            logger.info("Path found, heading to blue")
            self.servo.set_angle(pan_angle_to_best_blob)
            self.servo.sleep_ms_simulated(500) 
            
            self.turn_to_blob(self.blue_id, pan_angle_to_best_blob, self.find_dir_from_angle(pan_angle_to_best_blob))

    def find_dir_from_angle(self, angle) -> None:

        """
        find which way to turn, depending on the pan angle, in order to get the blob to the middle of the robot
        """
        logger.debug("Calculating turn direction")
        logger.info("finding direction")
        return 1 if angle < 0 else -1

    def turn_to_blob(self, blob_id : int, angle: float, direction: float) -> None:

        """
        Turns to blob with the body, to straighten the camera but keep blob in frame

        Args: angle: the angle to turn
            direction: which way to turn
        """
        logger.debug("Initiating turn to blob")
        logger.info("turn to blob called")
#        print(f"Blob ID is {blob_id}, angle is {angle}, direction is {direction}")
        # if abs(angle)>34:
        #     print ("USING THE BOOST")
        #     self.drive(0.15, direction*-0.3)
        #     self.servo.sleep_ms_simulated(400) 

        #     self.drive(0, 0)
        #     # time.sleep_ms(50)

        blobs, img = self.cam.get_blobs_bottom(blob_id)
        blob_found = self.cam.get_biggest_blob(blobs)

        # if blob_found is not None:
        #     print("found blue checking for valid")
        #     blob_found = self.is_valid(blob_found)
        # Try to line up the body with the camera

        if blob_found is not None:
            logger.info("Valid blue blob detected")
            self.drive(0.05, direction)
            while ((abs(angle) > 5) and (abs(angle) < 45)):
                logger.debug("Turning to align with blob")
                logger.info("turn to blob loop entered")
                blobs, img = self.cam.get_blobs_bottom(blob_id)
                blob_found = self.cam.get_biggest_blob(blobs)
    
                # if blob_found is not None:
                #     blob_found = self.is_valid(blob_found)

                if blob_found is not None:
                    angle = self.track_blob(blob_found)
        
                    self.servo.sleep_ms_simulated(50) 

                else:
                    self.drive(0,0) # Stop spinning
                    break
            else:
                logger.info(f"No Valid blue blob detected from {self.state.get_gaze()}")
        self.drive(0,0) # Stop spinning


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
        logger.info(f"Tracking metrics - blob cx {blob.cx()}, pixel_error: {pixel_error}, angle_error: {angle_error}, pid_error: {pid_error}, pan_angle: {pan_angle}")
        

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

    def is_new_blob_better_than_old(self, new_blob, old_blob, new_angle, old_angle):
        if old_blob is None:
            return new_blob, new_angle
        else: # TODO: Add in north star proximity
            if(new_blob.pixels() > old_blob.pixels()):
                logger.debug(f"better angle, {new_angle}")
                return new_blob, new_angle
            else:
                return old_blob, old_angle


    def dead_end_exit(self):
        self.servo.set_angle(0) # maybe do a scan first
        self.servo.sleep_ms_simulated(100) 

        biggest_star = None
        counter = 0
        # look for the star
        star_blobs, img = self.cam.get_blobs_bottom(self.star_id)
        biggest_star = self.cam.get_biggest_blob(star_blobs)
        while biggest_star is None and counter <16:
            logger.debug("Turning to locate star")
            self.servo.set_spin(0.1*self.star_is_right)
            self.servo.sleep_ms_simulated(100) 

            self.drive(0,0)
            self.servo.sleep_ms_simulated(100) 

            star_blobs, img = self.cam.get_blobs_bottom(self.star_id)
            biggest_star = self.cam.get_biggest_blob(star_blobs)
            counter +=1
        logger.info("Star located")
        if biggest_star is not None:
            if (self.cam.w_centre-biggest_star.cx()>0):
                self.star_is_right = 1
            else:
                self.star_is_right = -1
            while True:
                logger.debug("Executing escape turn")
                green_blobs, img = self.cam.get_blobs_bottom(self.green_id)
                biggest_green = self.cam.get_biggest_blob(green_blobs)

                if biggest_green is not None:
    #                print (" green found")
                    # biggest_green = self.is_valid(biggest_green)
                    if (biggest_green is not None):
                        logger.info("Valid green path found during escape")
                        return

                blue_blobs, img = self.cam.get_blobs_bottom(self.blue_id)
                biggest_blue = self.cam.get_biggest_blob(blue_blobs)


    #             if biggest_blue is not None:
    # #                    print (" blue found")
    #                     biggest_blue = self.is_valid(biggest_blue)
                if biggest_blue is not None:
                    logger.info("Valid blue path found during escape")
                    return

                self.servo.set_spin(0.1*self.star_is_right)
                self.servo.sleep_ms_simulated(100)
                self.drive(0,0)
                self.servo.sleep_ms_simulated(100)

                logger.debug("No valid path found during escape turn")
                print("no blob found")




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


    # def reset(self) -> None:
    #     self.servo.soft_reset()


    # def release(self) -> None:
    #     self.servo.release_all()

    def is_valid(self, biggest_blue_blob):
            return biggest_blue_blob
#             ## change biggest_blue_blob bottom coordinates
#             delta_y = self.cam.h_center-(biggest_blue_blob.cy() + biggest_blue_blob.h()/2)
#             beta = delta_y/(self.cam.h_center*2) * self.cam.v_fov
#             distance = abs( self.camera_height /math.tan((self.alpha + beta)*math.pi/180))

#             if distance > 40:
# #                print("Biggest blue blob found is too far away. Move not valid.")
#                 return None
#             else:
#                 if(biggest_blue_blob.pixels() > 300):
# #                    print("Biggest blue blob <25 and passes pixel threshold.")
#                     return biggest_blue_blob
#                 else:
# #                    print("Biggest blue blob <25 and fails pixel threshold.")
#                     return None
