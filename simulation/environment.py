import logging
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math
import matplotlib.transforms as mtransforms
from matplotlib.animation import FuncAnimation
import csv
import time
from robot_sim import *
import pandas as pd
from visualization import EnvironmentVisualizer

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
    
    def update_detection_probability(self, robot_state, max_range, obstacles,cam,bottom_fraction=1):
        """
        Update the probability that this cell is detected by the robot based on
        its position relative to the robot and the robot's gaze direction.
        """
        self.detection_probability = 1
        sensor_width =cam.w_centre * 2 # Width of the camera sensor in pixels
        sensor_height = cam.h_centre * 2  # Height of the camera sensor in pixels
        camera_h = cam.camera_h
        cam_elevation_angle = cam.camera_elevation_angle
        v_fov = cam.v_fov
        h_fov =  cam.h_fov
        gaze_dir = robot_state.theta + robot_state.phi
        gaze_dir = (gaze_dir + 90) % 360
        if gaze_dir > 180:
                gaze_dir -= 360  # Normalize to [-180, 180]

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
    
        self._pixels = max(0,  1 - (flat_distance - min_distance +  half_square_width)/(max_range-(min_distance+half_square_width))) # max pixels = 100
        angle_to_closest_point_vertical = math.degrees(math.atan2(camera_h, flat_distance-half_square_width)) *-1 
        angle_to_farthest_point_vertical = math.degrees(math.atan2(camera_h, flat_distance+half_square_width)) *-1 
        if angle_to_farthest_point_vertical< cam_elevation_angle-v_fov/2 or angle_to_closest_point_vertical> cam_elevation_angle+v_fov/2:
            self.detection_probability = 0.0
            self._pixels = 0
            return
        elif abs(angle_to_closest_point_vertical)>abs(cam_elevation_angle-v_fov/2):
            self._pixels = self._pixels * (1-(abs(angle_to_closest_point_vertical)-abs(cam_elevation_angle-v_fov/2))/(abs(angle_to_closest_point_vertical)-abs(angle_to_farthest_point_vertical)))
        
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
            self.detection_probability = 0.0
            self._pixels = 0
            # print(f"Detection 0 for cell ({self.x}, {self.y}): Cell is outside horizontal FOV. beta: {beta}, smallest angle: {leftmost_angle}, largest angle: {rightmost_angle}")
            return  # Cell is outside horizontal FOV
        elif max_angle > abs(beta):
            self._pixels = self._pixels * (1-(max_angle-beta)/(max_angle-min_angle))
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
                    self.detection_probability = 0
                    return
                elif leftmost_angle <= leftmost_obs_angle and leftmost_obs_angle<= rightmost_angle:
                    right_overlap = abs(leftmost_obs_angle-rightmost_angle)/abs(leftmost_angle-rightmost_angle)
                    max_right_overlap = max(right_overlap,max_right_overlap)
  
                elif leftmost_angle <= rightmost_obs_angle and rightmost_obs_angle <= rightmost_angle:
                    left_overlap = abs(leftmost_angle-rightmost_obs_angle)/abs(leftmost_angle-rightmost_angle)
                    max_left_overlap= max(max_left_overlap,left_overlap)
            
        self._pixels = self._pixels * (1-max_left_overlap) * (1-max_right_overlap)
        if self._pixels == 0:
            self.detection_probability = 0
        

        self._cx = (sensor_width / 2) * (1 + relative_angle_horizontal/ beta)
        if self._cx>sensor_width:
            self._cx = sensor_width
        elif self._cx<-sensor_width:
            self._cx = -   sensor_width
        self._cy = (sensor_height / 2) * (1 + math.radians(angle_to_cell_vertical) / math.radians(v_fov / 2))
        
# Class for obstacles
class Star(DetectableObject):
    def __init__(self, position):
        self.positiom = position
        self.color_id = 3

    def update_detection_probability(self, robot_state, max_range, obstacles, cam,bottom_fraction=1):
        """
        Calculate detection probability for obstacles (based on height and dimensions).
        """
        self.x = 0
        self.y = 0
        self.detection_probability = 0
        self._pixels = 0
        return 
    
class Obstacle(DetectableObject):
    def __init__(self, x, y, height, color_id=2):
        super().__init__(x, y, color_id)
        self.height = height

    def update_detection_probability(self, robot_state, max_range, obstacles, cam,bottom_fraction=1):
        """
        Calculate detection probability for obstacles (based on height and dimensions).
        """

        self.detection_probability = 0
        self._pixels = 0
        return 

# The environment and simulation class
class Environment:
    def __init__(self, task="RoboNav", starting_pos=(0, 0, 0, 0), target_pos=(1, 5), obstacles=[],star_position = 1, log_level=logging.INFO):
        # Set logging level
        logger.setLevel(log_level)
        
        # Set grid
        if task == "RoboNav":
            self.grid_size = [4, 6]
            self.square_size = 17
        self.starting_pos = starting_pos  # Starting position
        self.target = target_pos
        self.obstacles = []
        self.robot = Robot(self)
        self.star = Star(position = star_position)

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
        self.detectable_objects.append(self.star)
        self.show_frames = True

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

    def run_simulation(self, speed=0.2, bias=0, max_sim_time=5, time_step=0.02, task="RoboNav",show_frames = False):
        """
        Run the simulation for a specified amount of time.
        """
        # Set simulation parameters
        self.max_sim_time = max_sim_time
        self.delta_t = time_step
        self.sim_time = 0
        self.robot_states = []
        self.snapshot_data = []
        self.show_frames = show_frames
        if task == "RoboNav":
            self.robot.RoboNav(speed, bias)

        # Log data at the end of simulation
        self.save_to_csv()

    def take_snapshot(self, robot_state, max_range=45, bottom_fraction=1):
        """
        Update detection probabilities for all detectable objects.
        """
        if hasattr(self, "max_sim_time"):
            if self.sim_time >= self.max_sim_time:
                self.save_to_csv()
                raise TimeoutError("Simulation time exceeded")

        # Update detection for all objects
        for obj in self.detectable_objects:
            obj.update_detection_probability(robot_state, max_range, self.obstacles, self.robot.cam, bottom_fraction)

        # Log detection probabilities
        if hasattr(self, "max_sim_time"):
            self.log_detection_probabilities()
            
        # Modify this part to use the visualizer
        if self.show_frames:
            self.visualizer.draw_view(robot_state)
            
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
            fieldnames = ['time', 'x', 'y', 'type', 'detection_probability',"pixels"]
            writer = csv.DictWriter(snapshot_file, fieldnames=fieldnames)
            writer.writeheader()
            for snapshot in self.snapshot_data:
                writer.writerow(snapshot)