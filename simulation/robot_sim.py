from servos_sim import *
from camera_sim import *
from pid_control_sim import PID
import math
import numpy as np
import matplotlib.pyplot as plt
import copy


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
        self.servo = Servo(self)
        # self.servo.soft_reset()
        self.cam = Cam(self,env) # 
        self.PID = PID(self,p, i, d, imax)
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
        # Initiates
    
        target_has_been_found = False # variable for the green found
        target_reached = False # variable for when green was found, but now is lost, so target is reached

        while target_reached is False:
            blue_found = self.is_blob_in_frame(self.blue_id)
            green_found = self.is_blob_in_frame(self.green_id)

            # Check if time to completely stop
            if target_has_been_found is True: #check if the robot has finished
                print("Target has been found")
                if green_found is False: #if no green, green is already passed, so finish
                    print("the target has been found but now is lost, break while loop")
                    target_reached = True
                    #extra drive to get into the square with the body
                    self.drive(0.2, 0)
                    self.servo.sleep_ms_simulated(700)  #800 if 0.1 speed with wire
                    self.drive(0, 0)

                elif green_found is True: #check if it needs to go to green
                    print('Target found')
                    target_has_been_found = True
                    target_reached = self.go_to_blob(speed, bias, self.green_id)

            elif green_found is True: #check if it needs to go to green
                print('Target found')
                target_has_been_found = True
                target_reached = self.go_to_blob(speed, bias, self.green_id)


            elif blue_found is True: #check if it needs to go to blue
                print('Blue found')
                target_has_been_found = self.go_to_blob(speed, bias, self.blue_id)

            else: #if nothing has been found, look for blobs
                print('Nothing found, looking')
                self.searching_manager()
                # Scan: pan the camera to find blobs
                # If scan success
        return 


    def is_blob_in_frame(self, index: int) -> bool:
        """
        Checks if there is a blob if a colour in frame
        RETURN: bool
        """
        print ("is blob in frame")
        blobs, img = self.cam.get_blobs_bottom(index)
        found_blue = self.cam.get_biggest_blob(blobs)


        # if found_blue is not None:
        #     found_blue = self.is_valid(found_blue)
        return True if found_blue is not None else False

    def wait_for_camera(self) -> None:
        """
        buffer for the camera gain to start working
        """
        print("waiting for camera")
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
                print ("looking for green")
                #look for green in every loop, in case green appears while going to blue
                green_blobs, img = self.cam.get_blobs_bottom(self.green_id)
                found_biggest_green = self.cam.get_biggest_blob(green_blobs)

                # if found_biggest_green is not None:
                #     print("found green, check if valid")
                #     found_biggest_green = self.is_valid(found_biggest_green)

                if found_biggest_green is not None:
                    print("found green")
                    self.drive(0, 0)
                    print ("Break and switch ids")
                    return True

            # if no green found, look for the next blob (depending on blob_id)
            print ("go to blob  " + str(counter))
            blobs, img = self.cam.get_blobs_bottom(blob_id)
            found_biggest = self.cam.get_biggest_blob(blobs)

            # if found_biggest is not None:
            #     found_biggest = self.is_valid(found_biggest)

            if found_biggest is not None:
                print("Drive to assigned blob")
                counter=0
                print(f" found_biggest.cx {found_biggest.cx()}, id {found_biggest.color_id}")
                print(f" self.cam.w_centre {self.cam.w_centre}")
                steering = round((self.cam.w_centre-found_biggest.cx())/(2*self.cam.w_centre),3) 
                print(f"steering: {steering}")
                self.drive(speed, steering)
                self.servo.sleep_ms_simulated(50)  
                
               
            else:
                print ("lost assigned blob" + str(counter))
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
        print ("Looking for blob")
        self.drive(0, 0)
        pan_angle_to_best_blob, is_best_blob_green = self.scan_for_blob(step=2, limit=40)

        if (is_best_blob_green is True):
            print("Heading to green")
            self.servo.set_angle(pan_angle_to_best_blob)
            self.servo.sleep_ms_simulated(200)

            self.turn_to_blob(self.green_id, pan_angle_to_best_blob, self.find_dir_from_angle(pan_angle_to_best_blob))
        elif (pan_angle_to_best_blob is None):
            print('reach deadend')
            print("dead end")
            self.dead_end_exit()
        else:
            print("Heading to blue")
            self.servo.set_angle(pan_angle_to_best_blob)
            self.servo.sleep_ms_simulated(200) 
            
            self.turn_to_blob(self.blue_id, pan_angle_to_best_blob, self.find_dir_from_angle(pan_angle_to_best_blob))

    def find_dir_from_angle(self, angle) -> None:

        """
        find which way to turn, depending on the pan angle, in order to get the blob to the middle of the robot
        """
        print ("finding direction")
        return 1 if angle < 0 else -1

    def turn_to_blob(self, blob_id : int, angle: float, direction: float) -> None:

        """
        Turns to blob with the body, to straighten the camera but keep blob in frame

        Args: angle: the angle to turn
            direction: which way to turn
        """
        print ("turn to blob called")
#        print(f"Blob ID is {blob_id}, angle is {angle}, direction is {direction}")
        # if abs(angle)>34:
        #     print ("USING THE BOOST")
        #     self.drive(0.15, direction*-0.3)
        #     self.servo.sleep_ms_simulated(400) 

        #     self.drive(0, 0)
#            time.sleep_ms(50)

        blobs, img = self.cam.get_blobs_bottom(blob_id)
        blob_found = self.cam.get_biggest_blob(blobs)

        # if blob_found is not None:
        #     print("found blue checking for valid")
        #     blob_found = self.is_valid(blob_found)
        # Try to line up the body with the camera

        if blob_found is not None:
            print("BLUE valid")
            self.drive(0.05, direction)
            while ((abs(angle) > 5) and (abs(angle) < 45)):
                print("turn to blob loop entered")
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
        print(f"pixel_error: {pixel_error}, angle_error:m{angle_error}, pid_error: {pid_error}, pan_angle:{pan_angle} ")

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
                print("star found in scanning")
                if new_pan_angle>=0:
                    self.star_on_right = -1
                else:
                    self.star_on_right = 1

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                counter += 1
                print("Not found ...")
                self.scan_direction *= -1
#        print (pan_angle_to_best_blob)
        return pan_angle_to_best_blob, is_best_blob_green

    def is_new_blob_better_than_old(self, new_blob, old_blob, new_angle, old_angle):
        if old_blob is None:
            return new_blob, new_angle
        else: # TODO: Add in north star proximity
            if(new_blob.pixels() > old_blob.pixels()):
                print(f"better angle, {new_angle}")
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
            print("turning to star")
            self.servo.set_spin(0.1*self.star_is_right)
            self.servo.sleep_ms_simulated(100) 

            self.drive(0,0)
            self.servo.sleep_ms_simulated(100) 

            star_blobs, img = self.cam.get_blobs_bottom(self.star_id)
            biggest_star = self.cam.get_biggest_blob(star_blobs)
            counter +=1
        print("star found")
        if biggest_star is not None:
            if (self.cam.w_centre-biggest_star.cx()>0):
                self.star_is_right = 1
            else:
                self.star_is_right = -1
            while True:
                print("turning to escape")
                green_blobs, img = self.cam.get_blobs_bottom(self.green_id)
                biggest_green = self.cam.get_biggest_blob(green_blobs)

                if biggest_green is not None:
    #                print (" green found")
                    # biggest_green = self.is_valid(biggest_green)
                    if (biggest_green is not None):
                        print ("Valid green found")
                        return

                blue_blobs, img = self.cam.get_blobs_bottom(self.blue_id)
                biggest_blue = self.cam.get_biggest_blob(blue_blobs)


    #             if biggest_blue is not None:
    # #                    print (" blue found")
    #                     biggest_blue = self.is_valid(biggest_blue)
                if biggest_blue is not None:
                    print ("Valid blue found")
                    return

                self.servo.set_spin(0.1*self.star_is_right)
                self.servo.sleep_ms_simulated(100)
                self.drive(0,0)
                self.servo.sleep_ms_simulated(100)

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
