from environment import *
import logging

logger = logging.getLogger(__name__)
class Cam(object):
    """
    The Cam class manages the camera sensor for image capturing, processing,
    and color tracking. It initializes the camera parameters and sets the color
    thresholds for blob detection.
    """

    def __init__(self,robot,env):
        """
        Initialise the Cam object by setting up camera parameters and
        configuring color thresholds.
        """
        self.robot = robot
        self.env = env
        self.frame_delay = 0 

        # Initialise sensor properties - can edit these
        self.w_centre = 320 # pixels
        self.h_centre = 240 # pixels
        self.h_fov = 31.5 # degrees
        self.v_fov = 21 # degrees
        self.camera_elevation_angle = -15# degrees
        self.camera_h = 7 # cm
        self.max_distance = 40 # cm 
        self.min_distance = 10 # cm

    def set_frame_delay(self, frame_buffer):
        if frame_buffer:
            self.frame_delay = 0.02 # time it takes to take a frame when frame_buffer is on
        else:
            self.frame_delay = 0.001 # time it takes to take a frame when frame_buffer is off

    def get_blobs(self, index = None, angle = 0) -> tuple:
        """
        Capture an image and detect color blobs based on predefined thresholds.

        Returns:
            blobs (list): List of detected blobs.22
            img (image): Captured image used to find blobs.
        """
        blobs, img = self.env.take_snapshot(self.robot.state, frame_delay = self.frame_delay)
        if index is not None:
            blobs = [blob for blob in blobs if blob.detected == 1 and blob.color_id == index]
        else:
            blobs = [blob for blob in blobs if blob.detected == 1 and blob.color_id in [0, 1, 2, 3]]

        return blobs, img
     



    def get_blobs_bottom(self,index = None, angle = 0,max_range = 20) -> tuple:
        """
        Capture an image and detect color blobs based on predefined thresholds.

        Returns:
            blobs (list): List of detected blobs.
            img (image): Captured image used to find blobs.
        """

        blobs, img = self.env.take_snapshot(self.robot.state, max_range=max_range, frame_delay = self.frame_delay)
    
    
    
        if index is not None:
            blobs = [blob for blob in blobs if blob.detected == 1 and blob.color_id == index]
        else:
            blobs = [blob for blob in blobs if blob.detected == 1 and blob.color_id in [0, 1, 2, 3]]

        return blobs, img

    def get_biggest_blob(self, blobs):
        """
        Identify and return the largest blob from a list of detected blobs.

        Args:
            blobs (list): List of detected blobs.

        Returns:
            big_blob (blob): The biggest blob from list - see OpenMV docs for blob class.
        """
        max_pixels = 0
        big_blob = None

        for blob in blobs:
            # Update the big blob if the current blob has more pixels
            if blob.pixels() > max_pixels:
                max_pixels = blob.pixels()
                big_blob = blob

        return big_blob


    def get_blob_colours(self, blobs) -> int:
        """
        Returns the binary code (as int) of thresholds met by each blob

        Args:
            blobs (list): List of detected blobs.

        Returns:
            colours (list): List of binary codes (as int) of thresholds met by each element in blobs.
        """
        colours = []

        for blob in blobs:
            colours.append(blob.color_id)

        return colours


    def find_blob(self, blobs, threshold_idx: int):
        """
        Finds the first blob in blobs that was detected using a specified threshold

        Args:
            blobs (list): List of detected blobs.
            threshold_idx (int): Index along self.thresholds.

        Returns:
            found_idx (int): Index along blobs for the first blob that was detected using self.thresholds(threshold_idx)
        """
        colours = self.get_blob_colours(blobs)

        for found_idx, colour in enumerate(colours):
            if colour == threshold_idx:
                return found_idx

        return None