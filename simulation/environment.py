
import logging
# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
import numpy as np
import math
# import matplotlib.transforms as mtransforms
# from matplotlib.animation import FuncAnimation
import csv
import time
from robot_sim import *
# import pandas as pd
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

    def update_detection_probability(self, robot_state, max_range, obstacles, bottom_fraction=1):
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
       
    
    def update_detection_probability(self, robot_state, max_range, obstacles,cam,bottom_fraction=1,detection_factor = 1,max_speed = 20):
        """
        Update the probability that this cell is detected by the robot based on
        its position relative to the robot and the robot's gaze direction.
        """
        logger.debug(f"Processing cell: {self.x}, {self.y}, type: {self.color_id}")
        self.detection_probability = 0
        self.detected = 0
        sensor_width =cam.w_centre * 2 # Width of the camera sensor in pixels
        sensor_height = cam.h_centre * 2  # Height of the camera sensor in pixels
        max_pixels = (sensor_width * sensor_height) / 6
        camera_h = cam.camera_h
        cam_elevation_angle = cam.camera_elevation_angle
        v_fov = cam.v_fov
        h_fov =  cam.h_fov
        gaze_dir = robot_state.theta + robot_state.phi
        gaze_dir = (gaze_dir + 90) % 360
        if gaze_dir > 180:
                gaze_dir -= 360  # Normalize to [-180, 180]
        logger.debug(f"Robot state: {robot_state.x}, {robot_state.y}, gaze direction: {gaze_dir}")

        colored_square_side = 5
        half_square_width = colored_square_side / 2
        min_distance = abs(camera_h*math.tan(math.radians(cam_elevation_angle+v_fov/2+90)))
         # Calculate distance from robot to this cell
        beta = math.degrees(math.atan(math.tan(math.radians(h_fov/2))*math.sqrt(camera_h**2+min_distance**2)/min_distance))
        dx = self.x - robot_state.x # to centre of the cell
        dy = self.y - robot_state.y # to centre of the cell
        flat_distance = math.sqrt(abs(dx) ** 2 + abs(dy) ** 2)
        
        # Rotation matrix to rotate the grid cell coordinates by -gaze_dir around the robot's position
        cos_gaze = math.cos(math.radians(-gaze_dir))
        sin_gaze = math.sin(math.radians(-gaze_dir))

        # Apply the rotation matrix
        rotated_x = cos_gaze * dx - sin_gaze * dy
        rotated_y = sin_gaze * dx + cos_gaze * dy

        # Now compute the angle based on the rotated coordinates
        relative_angle_horizontal = math.degrees(math.atan2(rotated_y, rotated_x))
        angle_to_cell_vertical = math.degrees(math.atan2(camera_h, flat_distance))  # Vertical angle
        angle_to_cell_vertical = angle_to_cell_vertical*-1 - cam_elevation_angle
        logger.debug(f"Angle to cell: {relative_angle_horizontal}, {angle_to_cell_vertical}")
        self._pixels = max(0,  max_pixels *(1 - (flat_distance - min_distance +  half_square_width)/(max_range-(min_distance+half_square_width)))) 
        logger.debug(f"Pixels: {self._pixels}")
        self._pixels = min(max_pixels, self._pixels)
        logger.debug(f"Pixels: {self._pixels}")
        angle_to_closest_point_vertical = math.degrees(math.atan2(camera_h, flat_distance-half_square_width)) *-1 
        angle_to_farthest_point_vertical = math.degrees(math.atan2(camera_h, flat_distance+half_square_width)) *-1 
        if angle_to_farthest_point_vertical< cam_elevation_angle-v_fov/2 or angle_to_closest_point_vertical> cam_elevation_angle+v_fov/2:
            # self.detection_probability = 0.0
            self._pixels = 0
            logger.debug(f"Detection 0 for cell ({self.x}, {self.y}): Cell is outside vertical FOV. cam_elevation_angle: {cam_elevation_angle}, angle_to_closest_point_vertical: {angle_to_closest_point_vertical}, angle_to_farthest_point_vertical: {angle_to_farthest_point_vertical}")
            return
        elif abs(angle_to_closest_point_vertical)>abs(cam_elevation_angle-v_fov/2):
            self._pixels = self._pixels * (1-(abs(angle_to_closest_point_vertical)-abs(cam_elevation_angle-v_fov/2))/(abs(angle_to_closest_point_vertical)-abs(angle_to_farthest_point_vertical)))
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
            # self.detection_probability = 0.0
            self._pixels = 0
            logger.debug(f"Detection 0 for cell ({self.x}, {self.y}): Cell is outside horizontal FOV. beta: {beta}, smallest angle: {leftmost_angle}, largest angle: {rightmost_angle}")
            # print(f"Detection 0 for cell ({self.x}, {self.y}): Cell is outside horizontal FOV. beta: {beta}, smallest angle: {leftmost_angle}, largest angle: {rightmost_angle}")
            return  # Cell is outside horizontal FOV
        elif max_angle > abs(beta):
            self._pixels = self._pixels * (1-(max_angle-beta)/(max_angle-min_angle))
            self._pixels = min(max_pixels, self._pixels)
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
                    # self.detection_probability = 0
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
       
        

        self._cx = (sensor_width / 2) * (1 + relative_angle_horizontal/ beta) 
        if self._cx>sensor_width:
            self._cx = sensor_width
        elif self._cx<-sensor_width:
            self._cx = -   sensor_width
        self._cy = (sensor_height / 2) * (1 + math.radians(angle_to_cell_vertical) / math.radians(v_fov / 2))
        logger.debug(f"cx: {self._cx}, cy: {self._cy}")
        logger.debug(f"Pixels for cell ({self.x}, {self.y}): {self._pixels}")

        if self._pixels>0:
            if detection_factor ==1:
                self.detection_probability = 1
            else:
                speed = abs(robot_state.l_speed + robot_state.r_speed)/2
                normalized_speed = min(1.0, speed / max_speed)
                normalized_pixels = min(1.0, obj._pixels / (sensor_width * sensor_height/6)) # max detection when the object covers 1/6 of the frame
                self.detection_probability = detection_factor * (1-normalized_speed) * normalized_pixels
        self.detected = 1 if random.random() <= self.detection_probability else 0
    

class Star(DetectableObject):
    def __init__(self, position):
        super().__init__(x = 0, y = 0, color_id = 3)
        self.position = position


    def update_detection_probability(self, robot_state, max_range, obstacles, cam,bottom_fraction=1):
        """
        Calculate detection probability for obstacles (based on height and dimensions).
        """
        self.detection_probability = 0
        self.detected = 0
        self._pixels = 0
        return 
    
class Obstacle(DetectableObject):
    def __init__(self, x, y, height, color_id=2):
        super().__init__(x, y, color_id)
        self.height = height  # Height of the obstacle
        self.base_diameter = 5  # Diameter of the obstacle's base

    def update_detection_probability(self, robot_state, max_range, obstacles, cam, bottom_fraction=1,detection_factor = 1,max_speed = 20):
        """
        Calculate detection probability for obstacles (based on trapezoidal approximation) and update _cx, _cy.
        """
        logger.debug(f"Processing cell: {self.x}, {self.y}, type: {self.color_id}")
        # self.detection_probability = 0
        self.detection_probability = 0
        self.detected = 0
        self._pixels = 0
        self._cx = 0
        self._cy = 0

        # Camera and obstacle geometry
        sensor_width = cam.w_centre * 2
        sensor_height = cam.h_centre * 2
        max_pixels = sensor_width*sensor_height 
        camera_h = cam.camera_h
        cam_elevation_angle = cam.camera_elevation_angle
        v_fov = cam.v_fov
        h_fov = cam.h_fov
        gaze_dir = robot_state.theta + robot_state.phi
        gaze_dir = (gaze_dir + 90) % 360
        if gaze_dir > 180:
            gaze_dir -= 360  # Normalize to [-180, 180]
        logger.debug(f"Robot state: {robot_state.x}, {robot_state.y}, gaze direction: {gaze_dir}")
        # Distance and angular position relative to the robot
        dx = self.x - robot_state.x
        dy = self.y - robot_state.y
        flat_distance = math.sqrt(dx**2 + dy**2)

        # Check if the obstacle is within max range
        if flat_distance > max_range:
            logger.debug(f"Grid cell too far: {flat_distance} > {max_range}")
            return

        # Calculate angles to the obstacle
        center_angle_horizontal = math.degrees(math.atan2(dy, dx)) - gaze_dir
        if center_angle_horizontal > 180:
            center_angle_horizontal -= 360
        if center_angle_horizontal < -180:
            center_angle_horizontal += 360

        top_angle = math.degrees(math.atan2(self.height, flat_distance))
        bottom_angle = math.degrees(math.atan2(0, flat_distance)) - cam_elevation_angle

        # Check if the obstacle is within the FOV
        if abs(center_angle_horizontal) > h_fov / 2 or top_angle < cam_elevation_angle - v_fov / 2:
            logger.debug(f"Obstacle outside FOV: {center_angle_horizontal}, {top_angle}")
            return

        # Adjust visible height if it exceeds the FOV
        visible_top = min(top_angle, cam_elevation_angle + v_fov / 2)
        visible_bottom = max(bottom_angle, cam_elevation_angle - v_fov / 2)
        visible_height = max(0, visible_top - visible_bottom)
        logger.debug(f"Visible height: {visible_height}")
        # Adjust visible width if it exceeds the FOV
        left_angle = center_angle_horizontal - (self.base_diameter / (2 * flat_distance))
        right_angle = center_angle_horizontal + (self.base_diameter / (2 * flat_distance))
        visible_left = max(-h_fov / 2, left_angle)
        visible_right = min(h_fov / 2, right_angle)
        visible_width = max(0, visible_right - visible_left)
        logger.debug(f"Visible width: {visible_width}")
        # Calculate pixel representation
        # self._pixels = visible_height * (visible_width / h_fov)
        self._pixels = max_pixels * (visible_height / v_fov) * (visible_width / h_fov)
        self._pixels = min(max_pixels, self._pixels)
        logger.debug(f"Pixels: {self._pixels}")
        # Normalize the pixel value to a probability
        # self.detection_probability = min(1.0, max(0.0, self._pixels / (sensor_width * sensor_height)))

        # Calculate _cx and _cy (center of the obstacle in the image frame)
        self._cx = (sensor_width / 2) * (1 + center_angle_horizontal / (h_fov / 2))
        self._cy = (sensor_height / 2) * (1 + (cam_elevation_angle - (visible_top + visible_bottom) / 2) / (v_fov / 2))

        # Ensure _cx and _cy are within the image bounds
        self._cx = max(0, min(sensor_width, self._cx))
        self._cy = max(0, min(sensor_height, self._cy))
        logger.debug(f"cx: {self._cx}, cy: {self._cy}")

        if self._pixels>0:
            if detection_factor ==1:
                self.detection_probability = 1
            else:
                speed = abs(robot_state.l_speed + robot_state.r_speed)/2
                normalized_speed = min(1.0, speed / max_speed)
                normalized_pixels = min(1.0, obj._pixels / (sensor_width * sensor_height/2)) # max detection when the object covers 1/6 of the frame
                self.detection_probability = detection_factor * (1-normalized_speed) * normalized_pixels
        self.detected = 1 if random.random() <= self.detection_probability else 0


# The environment and simulation class
class Environment:
    def __init__(self, task="RoboNav", starting_pos=(0, 0, 0, 0), target_pos=(1, 5), obstacles=[],star_position = 1, log_level=logging.INFO,detection_factor = 1,servo_error = 0, pan_error = 0):
        # Set logging level
        logger.setLevel(log_level)
        
        # Set grid
        if task == "RoboNav":
            self.grid_size = [4, 6]
            self.square_size = 17
        self.starting_pos = starting_pos  # Starting position
        self.target = target_pos
        self.obstacles = []
        self.servo_error = servo_error
        self.pan_error = pan_error
        self.robot = Robot(self)
        self.star = Star(position = star_position)
        self.detection_factor = detection_factor

        # Initialize grid cells
        self.grid = np.empty(self.grid_size, dtype=object)
        for i in range(self.grid_size[0]):
            for j in range(self.grid_size[1]):
                x = i * self.square_size + self.square_size / 2
                y = j * self.square_size + self.square_size / 2
                if (i,j) in obstacles:
                    self.grid[i, j] = Obstacle(x, y, height=5)
                    self.obstacles.append(self.grid[i, j])
                else:
                    color_id = self.set_cell_id(i, j)
                    self.grid[i, j] = GridCell(x, y, color_id)

        # List of all detectable objects
        self.detectable_objects = []
        self.detectable_objects.extend(self.grid.flatten())
        self.detectable_objects.extend(self.obstacles)
        # self.detectable_objects.append(self.star)
        # self.show_frames = True

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
        # self.set_speed(self.curr_l_speed, self.curr_r_speed,self.robot.cam.env.delta_t)
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

    def take_snapshot(self, robot_state, max_range=40, bottom_fraction=1, frame_delay = 0):
        """
        Update detection probabilities for all detectable objects.
        """
        if hasattr(self, "max_sim_time"):
            if self.sim_time >= self.max_sim_time:
                self.save_to_csv()
                raise TimeoutError("Simulation time exceeded")

        # Update detection for all objects
        for obj in self.detectable_objects:
            obj.update_detection_probability(robot_state, max_range, self.obstacles, self.robot.cam, bottom_fraction,detection_factor = self.detection_factor,max_speed = self.robot.servo.max_speed)

        # Log detection probabilities
        self.log_detection_probabilities()
     
        self.update_time(frame_delay)
        # self.sim_time+= 0.001 # not tested yet, random value
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