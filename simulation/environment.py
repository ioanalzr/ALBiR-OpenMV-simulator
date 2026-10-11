
import logging
import numpy as np
import math
import csv
import time
from robot_sim import *
from visualization import EnvironmentVisualizer
import random

# Configure logging
logging.basicConfig(
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    level=logging.INFO  # Default level, can be changed
)
logger = logging.getLogger(__name__)

# Base class for detectable objects

class DetectableObject:
    def __init__(self, x, y, color_id):
        self.x = x
        self.y = y
        self.color_id = color_id  # 0 for blue, 1 for green, 2 for red, 3 for star
        self.detection_probability = 0.0
        self.detected = 0 # 0 or 1
        self._pixels = 0
        self._cx = 0
        self._cy = 0

    def update_detection_probability(self, robot_state, max_range, obstacles):
        raise NotImplementedError("This method should be implemented in subclasses")
    
    def cx(self):
        return self._cx
    def cy(self):
        return self._cy
    def pixels(self):
        return self._pixels

# Class for the grid cells
class GridCell(DetectableObject):
    def __init__(self, x, y, color_id):
        super().__init__(x, y, color_id)
        self.square_side = 3.5
       
    
    def update_detection_probability(self, robot_state, max_range, obstacles,cam,detection_factor = 1,max_speed = 2):
        """
        Update the probability that this cell is detected by the robot based on
        its position relative to the robot and the robot's gaze direction.
        """
        logger.debug(f"Processing cell: {self.x}, {self.y}, type: {self.color_id}")
        self.detection_probability = 0
        self.detected = 0
        self._pixels = 0
        sensor_width =cam.w_centre * 2 # Width of the camera sensor in pixels
        sensor_height = cam.h_centre * 2  # Height of the camera sensor in pixels
        max_pixels = (sensor_width * sensor_height) / 11
        min_pixels = (sensor_width * sensor_height) / 200
        camera_h = cam.camera_h
        cam_elevation_angle = cam.camera_elevation_angle
        v_fov = cam.v_fov
        h_fov =  cam.h_fov

        gaze_dir = robot_state.theta + robot_state.phi
        gaze_dir = (gaze_dir + 90) % 360
        if gaze_dir > 180:
                gaze_dir -= 360  # Normalize to [-180, 180]
        logger.debug(f"Robot state: {robot_state.x}, {robot_state.y}, gaze direction: {gaze_dir}")

        half_square_width = self.square_side / 2

        if cam.min_distance is None:
            min_distance = abs(camera_h*math.tan(math.radians(cam_elevation_angle-v_fov/2+90)))
        else:
            min_distance = cam.min_distance
        
        if max_range is None:
            max_range = cam.max_distance

        # Calculate distance from robot to this cell
        dx = self.x - robot_state.x # to centre of the cell
        dy = self.y - robot_state.y # to centre of the cell
        flat_distance = math.sqrt(abs(dx) ** 2 + abs(dy) ** 2)
        logger.debug(f"Flat distance: {flat_distance}")
        logger.debug(f"Max distance: {max_range}")
        logger.debug(f"Min distance: {min_distance}")
        if flat_distance > max_range:
            self._pixels = 0

        beta = math.degrees(math.atan(math.tan(math.radians(h_fov/2))*math.sqrt(camera_h**2+min_distance**2)/min_distance)) 
        
        # Rotation matrix to rotate the grid cell coordinates by -gaze_dir around the robot's position
        cos_gaze = math.cos(math.radians(-gaze_dir))
        sin_gaze = math.sin(math.radians(-gaze_dir))

        # Apply the rotation matrix
        rotated_x = cos_gaze * dx - sin_gaze * dy
        rotated_y = sin_gaze * dx + cos_gaze * dy

        # Compute the angle based on the rotated coordinates
        relative_angle_horizontal = math.degrees(math.atan2(rotated_y, rotated_x))
        angle_to_cell_vertical = math.degrees(math.atan2(camera_h, flat_distance))  # Vertical angle
        angle_to_cell_vertical = angle_to_cell_vertical*-1 - cam_elevation_angle
        logger.debug(f"Angle to cell: {relative_angle_horizontal}, {angle_to_cell_vertical}")
        
        a = (min_pixels - max_pixels) / ((max_range-half_square_width) - (min_distance + half_square_width))
        b = max_pixels - a * (min_distance + half_square_width)

        self._pixels = max(0, a * flat_distance + b)
        self._pixels = min(max_pixels, self._pixels)
        logger.debug(f"Pixels: {self._pixels}")
        
        d_cell_min = flat_distance - half_square_width
        d_cell_max = flat_distance + half_square_width

        # If the entire cell is outside the visible range, set pixels to zero.
        if d_cell_max < min_distance or d_cell_min > max_range:
            # The cell is completely outside the vertical field.
            self._pixels = 0

        # Determine the overlap between the cell's interval and the visible distance interval.
        visible_overlap = min(d_cell_max, max_range) - max(d_cell_min, min_distance)
        cell_interval = d_cell_max - d_cell_min

        # Compute the fraction of the cell that is visible.
        fraction_visible = visible_overlap / cell_interval

        # Scale the pixel count by the fraction visible.
        self._pixels = self._pixels * fraction_visible
        self._pixels = min(max_pixels, self._pixels)
        corners = [
            (self.x - half_square_width, self.y - half_square_width),  # bottom-left
            (self.x + half_square_width, self.y - half_square_width),  # bottom-right
            (self.x - half_square_width, self.y + half_square_width),  # top-left
            (self.x + half_square_width, self.y + half_square_width)   # top-right
        ]

        # Rotate corners around the center to the robot's perspective
        rotated_corners = []
        for cx, cy in corners:
            # Translate corner to the robot's reference frame (positioning it relative to the robot)
            rel_cx = cx - robot_state.x
            rel_cy = cy - robot_state.y
            
            # Angle from the robot to the corner
            angle_to_corner_rad = math.atan2(rel_cy, rel_cx)
            angle_to_corner_deg = math.degrees(angle_to_corner_rad)

            # Adjust the corner's angle with respect to the robot's gaze
            relative_angle_corner = (angle_to_corner_deg - gaze_dir  ) % 360
            if relative_angle_corner > 180:
                relative_angle_corner -= 360  # Normalize to [-180, 180]
            
            # Add the rotated corner's angle to the list
            rotated_corners.append(relative_angle_corner)

        # Check the field of view for the leftmost and rightmost points (corners)
        leftmost_angle = min(rotated_corners)
        rightmost_angle = max(rotated_corners)

        # Check if the corners are within the horizontal FOV
        min_angle = min(abs(leftmost_angle),abs(rightmost_angle))
        max_angle =  max(abs(leftmost_angle),abs(rightmost_angle))
        if min_angle > abs(beta):
            self._pixels = 0
            logger.debug(f"Detection 0 for cell ({self.x}, {self.y}): Cell is outside horizontal FOV. beta: {beta}, smallest angle: {leftmost_angle}, largest angle: {rightmost_angle}")
            return  # Cell is outside horizontal FOV
        elif max_angle > abs(beta):
            self._pixels = self._pixels * (1-(max_angle-beta)/(max_angle-min_angle))
            self._pixels = min(max_pixels, self._pixels)
        
        # Check for obstruction from obstacles
        max_right_overlap = 0
        max_left_overlap = 0
        half_obs_width =3
        for obs in obstacles:
            obs_dx = abs(obs.x - robot_state.x)
            obs_dy = abs(obs.y - robot_state.y)
            obs_flat_distance = math.sqrt(obs_dx ** 2 + obs_dy ** 2)

            if obs_flat_distance < flat_distance:
                # Find left and right points on the obstacle (perpendicular to the line from the robot to the obstacle)
                obs_direction = math.atan2(obs_dy, obs_dx)
                obs_left_x = obs.x + math.cos(obs_direction + math.pi / 2) * half_obs_width
                obs_left_y = obs.y + math.sin(obs_direction + math.pi / 2) * half_obs_width
                obs_right_x = obs.x + math.cos(obs_direction - math.pi / 2) * half_obs_width
                obs_right_y = obs.y + math.sin(obs_direction - math.pi / 2) * half_obs_width

                obs_left_angle = (math.degrees(math.atan2(obs_left_y - robot_state.y, obs_left_x - robot_state.x)) - gaze_dir) % 360
                if obs_left_angle > 180:
                    obs_left_angle -= 360  # Adjust to be between -180 and 180

                obs_right_angle = (math.degrees(math.atan2(obs_right_y - robot_state.y, obs_right_x - robot_state.x)) - gaze_dir) % 360
                if obs_right_angle > 180:
                    obs_right_angle -= 360  # Adjust to be between -180 and 180
                leftmost_obs_angle, rightmost_obs_angle = min(obs_left_angle, obs_right_angle), max(obs_left_angle, obs_right_angle)
                if leftmost_obs_angle <= leftmost_angle and rightmost_obs_angle>= rightmost_angle:
                    self._pixels = 0
                    return
                elif leftmost_angle <= leftmost_obs_angle and leftmost_obs_angle<= rightmost_angle:
                    right_overlap = abs(leftmost_obs_angle-rightmost_angle)/abs(leftmost_angle-rightmost_angle)
                    max_right_overlap = max(right_overlap,max_right_overlap)
  
                elif leftmost_angle <= rightmost_obs_angle and rightmost_obs_angle <= rightmost_angle:
                    left_overlap = abs(leftmost_angle-rightmost_obs_angle)/abs(leftmost_angle-rightmost_angle)
                    max_left_overlap= max(max_left_overlap,left_overlap)
        logger.debug(f"max_left_overlap: {max_left_overlap}, max_right_overlap: {max_right_overlap}")
        self._pixels = self._pixels * (1-max_left_overlap) * (1-max_right_overlap)
        self._pixels = min(max_pixels, self._pixels)
       

        if self._pixels>0:
            # Calculate _cx and _cy (center of the gridcell in the image frame)
            # Mirror _cx horizontally
            self._cx = sensor_width * (1 - ((1 + relative_angle_horizontal / beta) / 2))  # for mirror
            self._cx = max(0, min(sensor_width, self._cx))

            # Flip _cy vertically (0 at the top, sensor_height at the bottom)
            self._cy = ((v_fov / 2) - (-math.degrees(math.atan(camera_h / flat_distance))-cam_elevation_angle)) / v_fov *sensor_height
            self._cy = max(0, min(sensor_height, self._cy))
            

            # add cropping from rotation correction, not sure how well it works ----------------------------------------------------
            if robot_state.phi!=0:
                phi_rad = math.radians(abs(robot_state.phi) )

                # Compute the new bounding quadrilateral 
                cropped_width = sensor_width * abs(math.cos(phi_rad)) + sensor_height * abs(math.sin(phi_rad))
                cropped_height = sensor_height * abs(math.cos(phi_rad)) + sensor_width * abs(math.sin(phi_rad))

                # Ensure the cropped dimensions do not exceed the original sensor size
                cropped_width = min(sensor_width, cropped_width)
                cropped_height = min(sensor_height, cropped_height)
                
                # Define the **inscribed quadrilateral** center bounds
                cx_center = cam.w_centre  # Image center x
                cy_center = cam.h_centre  # Image center y

                # Compute the distance of the object center from the image center
                distance_to_center = math.sqrt((self._cx - cx_center) ** 2 + (self._cy - cy_center) ** 2)

                # Compute the max inscribed radius (half diagonal of the cropped region)
                max_inscribed_radius = math.sqrt((cropped_width / 2) ** 2 + (cropped_height / 2) ** 2)

                # Check if the object is cropped
                if distance_to_center > max_inscribed_radius:
                    self._pixels = 0  # Object is cropped out completely
                

                # Adjust pixels near the edges u
                edge_factor = max(0, 1 - ((distance_to_center - max_inscribed_radius * 0.75) / (max_inscribed_radius * 0.25)))
                edge_factor = min(1, edge_factor)  # Clamp between 0 and 1

                self._pixels *= edge_factor  
            # ----------------------------------------------------------------------------------------

        if self._pixels>0:
            if detection_factor ==1:
                self.detection_probability = 1
            else:
                speed = abs(robot_state.l_speed + robot_state.r_speed)/2
                normalized_speed = min(1.0, speed / max_speed)
                # normalized_pixels = min(1.0, self._pixels / max_pixels)  # if you want detection probability to also depend on pixel number
                normalized_pixels = 1
                self.detection_probability = detection_factor * (1-normalized_speed) * normalized_pixels
            
        self.detected = 1 if random.random() <= self.detection_probability else 0
        if self.detected == 1:
            logger.debug(f"Detected target at cell at: {self.x}, {self.y} ({flat_distance}) with probability {self.detection_probability}, from {robot_state.x}, {robot_state.y}, pixels: {self._pixels}, {self.cx(), self.cy()}")
            
   

class Obstacle(DetectableObject):
    def __init__(self, x, y, color_id=2):
        super().__init__(x, y, color_id)
        self.height = 5  # Height of the obstacle
        self.base_diameter = 5  # Diameter of the obstacle's base
        self.top_diameter = 3.5

    
    def update_detection_probability(self, robot_state, max_range, obstacles, cam, detection_factor=1, max_speed=2):
        """
        Update detection probability for an obstacle.
        """
    
        logger.debug(f"Processing obstacle at ({self.x}, {self.y})")

        self.detection_probability = 0
        self.detected = 0
        self._pixels = 0
        self._cx = 0
        self._cy = 0

        # Sensor and camera parameters.
        sensor_width = cam.w_centre * 2
        sensor_height = cam.h_centre * 2
        max_pixels = sensor_width * sensor_height * 2/3

        camera_h = cam.camera_h                
        cam_elev = cam.camera_elevation_angle    
        v_fov = cam.v_fov                   
        h_fov = cam.h_fov                  

        if cam.min_distance is None:
            min_distance = abs(camera_h*math.tan(math.radians(cam_elev-v_fov/2+90)))
        else:
            min_distance = cam.min_distance
        
        if max_range is None:
            max_range = cam.max_distance
            
        # Compute robot's gaze direction relative to world frame
        gaze_dir = robot_state.theta + robot_state.phi
        gaze_dir = (gaze_dir + 90) % 360
        if gaze_dir > 180:
            gaze_dir -= 360

        # Compute flat distance from robot to obstacle
        dx = self.x - robot_state.x
        dy = self.y - robot_state.y
        flat_distance = math.sqrt(dx**2 + dy**2)
        if flat_distance > max_range:
            return  # Obstacle is too far to be detected

        # Horizontal Angle 
        center_angle_horizontal = math.degrees(math.atan2(dy, dx)) - gaze_dir
        if center_angle_horizontal > 180:
            center_angle_horizontal -= 360
        if center_angle_horizontal < -180:
            center_angle_horizontal += 360
        if abs(center_angle_horizontal) > h_fov / 2:
            return  # Obstacle is outside horizontal FOV

        # Vertical Angles 
        obstacle_height = self.height  # 5 cm
        base_diameter = self.base_diameter  # 5 cm
        top_diameter  = self.top_diameter # 3.5 cm

        # Compute angle to the bottom 
        angle_bottom = -math.degrees(math.atan2(camera_h, flat_distance))
        # Compute angle to the top 
        angle_top = -math.degrees(math.atan2(camera_h - obstacle_height, flat_distance))
        rel_angle_bottom = angle_bottom - cam_elev
        rel_angle_top    = angle_top - cam_elev
        # Ensure that rel_angle_top is the larger value
        if rel_angle_top < rel_angle_bottom:
            rel_angle_top, rel_angle_bottom = rel_angle_bottom, rel_angle_top

        visible_top    = min(rel_angle_top, v_fov / 2)
        visible_bottom = max(rel_angle_bottom, -v_fov / 2)
        visible_vertical = max(0, visible_top - visible_bottom)
        fraction_vertical = visible_vertical / v_fov


        effective_diameter = (base_diameter + top_diameter) / 2  
        effective_radius = effective_diameter / 2
    
        horizontal_angle = 2 * math.degrees(math.atan(effective_radius / flat_distance))
        fraction_horizontal = horizontal_angle / h_fov

        # Projected Pixel Area 

        self._pixels = max_pixels * fraction_vertical * fraction_horizontal
        self._pixels = min(max_pixels, max(0, self._pixels))

        current_left = center_angle_horizontal - (horizontal_angle / 2)
        current_right = center_angle_horizontal + (horizontal_angle / 2)
        max_left_occlusion = 0
        max_right_occlusion = 0

        for obs in obstacles:
            if obs is self:
                continue  # Skip self
            # Compute occluder distance
            dx_o = obs.x - robot_state.x
            dy_o = obs.y - robot_state.y
            d_o = math.sqrt(dx_o**2 + dy_o**2)
            if d_o >= flat_distance:
                continue  # Only consider obstacles that are closer
            # Compute occluder's center angle relative to robot's gaze
            occ_center = math.degrees(math.atan2(dy_o, dx_o)) - gaze_dir
            if occ_center > 180:
                occ_center -= 360
            if occ_center < -180:
                occ_center += 360

            # Determine occluder effective radius
            if hasattr(obs, "base_diameter") and hasattr(obs, "top_diameter"):
                occ_eff_diam = (obs.base_diameter + obs.top_diameter) / 2
                occ_eff_radius = occ_eff_diam / 2
            else:
                occ_eff_radius = effective_radius  # Fall back to same as current obstacle

            occ_half_angle = math.degrees(math.atan(occ_eff_radius / d_o))
            occ_left = occ_center - occ_half_angle
            occ_right = occ_center + occ_half_angle

            # If the occluder completely covers the current obstacle's horizontal span, cancel detection
            if occ_left <= current_left and occ_right >= current_right:
                self._pixels = 0
                break

            # Otherwise, compute partial overlaps
            if occ_right > current_left and occ_right < current_right:
                overlap_left = occ_right - current_left
                left_fraction = overlap_left / (current_right - current_left)
                max_left_occlusion = max(max_left_occlusion, left_fraction)
            if occ_left < current_right and occ_left > current_left:
                overlap_right = current_right - occ_left
                right_fraction = overlap_right / (current_right - current_left)
                max_right_occlusion = max(max_right_occlusion, right_fraction)

        self._pixels *= (1 - max_left_occlusion) * (1 - max_right_occlusion)
        self._pixels = min(max_pixels, max(0, self._pixels))

 
        self._cx = sensor_width * (1 - (1 + center_angle_horizontal / (h_fov / 2)) / 2)
        self._cx = max(0, min(sensor_width, self._cx))
        
        avg_vertical = (visible_top + visible_bottom) / 2
        self._cy = sensor_height * (1 - (1 + avg_vertical / (v_fov / 2)) / 2)
        self._cy = max(0, min(sensor_height, self._cy))

        logger.debug(f"Obstacle pixels: {self._pixels:.2f}, cx: {self._cx:.2f}, cy: {self._cy:.2f}")

        # add cropping from rotation correction ----------------------------------------------------
        if robot_state.phi != 0:
            phi_rad = math.radians(abs(robot_state.phi))
            # Compute the dimensions of the inscribed quadrilateral
            cropped_width = sensor_width * abs(math.cos(phi_rad)) + sensor_height * abs(math.sin(phi_rad))
            cropped_height = sensor_height * abs(math.cos(phi_rad)) + sensor_width * abs(math.sin(phi_rad))
            # Ensure cropped dimensions do not exceed the sensor dimensions
            cropped_width = min(sensor_width, cropped_width)
            cropped_height = min(sensor_height, cropped_height)
            # Define the inscribed quadrilateral's center bounds
            cx_center = cam.w_centre
            cy_center = cam.h_centre
            # Compute distance from the obstacle's projected center to the image center
            distance_to_center = math.sqrt((self._cx - cx_center)**2 + (self._cy - cy_center)**2)
            # Maximum inscribed radius is half the diagonal of the cropped region.
            max_inscribed_radius = math.sqrt((cropped_width / 2)**2 + (cropped_height / 2)**2)
            # If the obstacle's center lies outside the inscribed region, it is fully cropped
            if distance_to_center > max_inscribed_radius:
                self._pixels = 0
            else:
                # For objects near the edge, gradually reduce the pixel count
                edge_factor = max(0, 1 - ((distance_to_center - max_inscribed_radius * 0.75) /
                                        (max_inscribed_radius * 0.25)))
                edge_factor = min(1, edge_factor)
                self._pixels *= edge_factor
        # ----------------------------------------------------
        logger.debug(f"Obstacle pixels: {self._pixels:.2f}, cx: {self._cx:.2f}, cy: {self._cy:.2f}")

        # Detection Probability 
        if self._pixels > 0:
            if detection_factor == 1:
                self.detection_probability = 1
            else:
                speed = abs(robot_state.l_speed + robot_state.r_speed) / 2
                normalized_speed = min(1.0, speed / max_speed)
                self.detection_probability = detection_factor * (1 - normalized_speed)

        self.detected = 1 if random.random() <= self.detection_probability else 0


# The environment and simulation class
class Environment:
    def __init__(self, task="RoboNav", starting_pos=(0, 0, 0, 0), target_pos=(1, 5), obstacles=[],star_position = 1, log_level=logging.INFO,detection_factor = 1,servo_error = 0, pan_error = 0,pan_delay = 0, speed_coef = 2):
        # Set logging level
        logger.setLevel(log_level)
        
        # Set grid
        if task == "RoboNav":
            self.grid_size = [4, 6]
            self.square_size = 18

        self.starting_pos = starting_pos  # Starting position
        self.target = target_pos
        self.obstacles = []
        self.servo_error = servo_error
        self.pan_error = pan_error
        self.pan_delay = pan_delay
        self.speed_coef = speed_coef
        self.robot = Robot(self)
        self.detection_factor = detection_factor
        

        # Initialize grid cells
        self.grid = np.empty(self.grid_size, dtype=object)
        for i in range(self.grid_size[0]):
            for j in range(self.grid_size[1]):
                x = i * self.square_size + self.square_size / 2
                y = j * self.square_size + self.square_size / 2
                if (i,j) in obstacles:
                    self.grid[i, j] = Obstacle(x, y)
                    self.obstacles.append(self.grid[i, j])
                else:
                    color_id = self.set_cell_id(i, j)
                    self.grid[i, j] = GridCell(x, y, color_id)

        # List of all detectable objects
        self.detectable_objects = []
        self.detectable_objects.extend(self.grid.flatten())
        self.detectable_objects.extend(self.obstacles)


        self.visualizer = EnvironmentVisualizer(self)

    def set_cell_id(self, i, j):
        """
        Initialize the grid cell with obstacles (color_id 2), target (color_id 1),
        or free space (color_id 0).
        """
        if (i, j) == self.target:
            return 1  # Target (green)
        return 0  # Free space (blue)

    def log_detection_probabilities(self):
        """
        Log the detection probabilities of all detectable objects.
        """
        for obj in self.detectable_objects:
            self.snapshot_data.append({
                'time': round(self.sim_time,3),
                'x': round(obj.x, 3),
                'y': round(obj.y, 3),
                'type': obj.color_id,  # Type of object (GridCell, Obstacle)
                'detection_probability': round(obj.detection_probability, 3),
                'detected': obj.detected,
                'pixels':  round(obj._pixels, 3)
            })

    def log_robot_state(self):
        """
        Log the robot's state (x, y, theta, phi).
        """
        self.robot_states.append({
            'time': round(self.sim_time, 3),
            'x': round(self.robot.state.x, 3),
            'y': round(self.robot.state.y, 3),
            'heading': round(self.robot.state.theta, 3),
            'gaze': round(self.robot.state.phi, 3)
        })

    def run_simulation(self, speed=0.2, bias=0, max_sim_time=5, time_step=0.02, task="RoboNav", frame_buffer = True):
        """
        Run the simulation for a specified amount of time.
        """
        # Set simulation parameters
        self.max_sim_time = max_sim_time
        self.delta_t = time_step
        self.sim_time = 0
        self.robot_states = []
        self.snapshot_data = []
        self.robot.cam.set_frame_delay(frame_buffer) 
        if task == "RoboNav":
            self.robot.RoboNav(speed, bias)

        # Log data at the end of simulation
        self.save_to_csv()

    def update_time(self, delta_t = None):
        if delta_t is None:
            delta_t = self.delta_t
        v_left,v_right = self.robot.state.get_speed()
        v = (v_left + v_right) / 2  # Linear velocity
        omega = (v_right - v_left) / self.robot.WHEEL_BASE  # Angular velocity

        # Update position and orientation
        delta_theta = math.degrees(omega * delta_t)
        
        delta_x = -v * math.sin(math.radians(self.robot.state.get_orientation())) * delta_t * 100  # Convert to cm 
        delta_y = v * math.cos(math.radians(self.robot.state.get_orientation())) * delta_t * 100 

        self.sim_time += delta_t
        # Update the robot's state (phi remains fixed unless changed externally)
        self.robot.state.update(delta_x, delta_y, delta_theta,0,v_left,v_right) ####
        self.log_robot_state()
        if self.sim_time >= self.max_sim_time:
            # log data
            self.robot.cam.env.save_to_csv()
            raise TimeoutError("Simulation time exceeded")
        return

    def take_snapshot(self, robot_state, max_range= None, frame_delay = 0):
        """
        Update detection probabilities for all detectable objects.
        """
        if hasattr(self, "max_sim_time"):
            if self.sim_time >= self.max_sim_time:
                self.save_to_csv()
                raise TimeoutError("Simulation time exceeded")

        # Update detection for all objects
        for obj in self.detectable_objects:
            obj.update_detection_probability(robot_state, max_range, self.obstacles, self.robot.cam, detection_factor = self.detection_factor,max_speed = self.robot.servo.max_speed)

        # Log detection probabilities
        self.log_detection_probabilities()
     
        self.update_time(frame_delay)
     
        return self.detectable_objects, self

    def save_to_csv(self):
        """
        Save robot states and detection probabilities to a CSV file.
        """
        # Save robot states
        with open('robot_states.csv', mode='w', newline='') as robot_file:
            fieldnames = ['time', 'x', 'y', 'heading', 'gaze']
            writer = csv.DictWriter(robot_file, fieldnames=fieldnames)
            writer.writeheader()
            for state in self.robot_states:
                writer.writerow(state)

        # Save detection probabilities (snapshots)
        with open('snapshots.csv', mode='w', newline='') as snapshot_file:
            fieldnames = ['time', 'x', 'y', 'type', 'detection_probability',"detected","pixels"]
            writer = csv.DictWriter(snapshot_file, fieldnames=fieldnames)
            writer.writeheader()
            for snapshot in self.snapshot_data:
                writer.writerow(snapshot)