import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math
import matplotlib.transforms as mtransforms
from matplotlib.animation import FuncAnimation
import csv
import time
from robot_sim import *

# Base class for detectable objects
class DetectableObject:
    def __init__(self, x, y, color_id):
        self.x = x
        self.y = y
        self.color_id = color_id  # 0 for blue, 1 for green, 2 for red, 3 for star
        self.detection_probability = 0.0

    def update_detection_probability(self, robot_state, max_range, obstacles, bottom_fraction=1):
        raise NotImplementedError("This method should be implemented in subclasses")


# Class for the grid cells
class GridCell(DetectableObject):
    def __init__(self, x, y, color_id):
        super().__init__(x, y, color_id)

    def update_detection_probability(self, robot_state, max_range, obstacles,cam,bottom_fraction=1):
        """
        Update the probability that this cell is detected by the robot based on
        its position relative to the robot and the robot's gaze direction.
        """
        self.pixels = 0
        self.detection_probability = 1
        sensor_width =cam.w_centre * 2 # Width of the camera sensor in pixels
        sensor_height = cam.h_centre * 2  # Height of the camera sensor in pixels
        camera_h = cam.camera_h
        cam_elevation_angle = cam.camera_elevation_angle
        v_fov = cam.v_fov
        h_fov =  cam.h_fov
        gaze_dir = robot_state.theta + robot_state.phi
        gaze_dir = (gaze_dir - 90) % 360
        if gaze_dir > 180:
                gaze_dir -= 360  # Normalize to [-180, 180]

        colored_square_side = 5
        half_square_width = colored_square_side / 2
        min_distance = abs(camera_h*math.tan(math.radians(cam_elevation_angle+v_fov/2+90)))
         # Calculate distance from robot to this cell
        beta = math.degrees(math.atan(math.tan(math.radians(h_fov/2))*math.sqrt(camera_h**2+min_distance**2)/min_distance))
        dx = abs(self.x - robot_state.x) # to centre of the cell
        dy = abs(self.y - robot_state.y) # to centre of the cell
        flat_distance = math.sqrt(dx ** 2 + dy ** 2)

      
      
        # relative_angle = (flat_angle_to_cell_deg - gaze_dir + 180) % 360
        angle_to_cell_horizontal = math.degrees(math.atan2(dy, dx))
        relative_angle_horizontal = (angle_to_cell_horizontal - gaze_dir + 180) % 360
        if relative_angle_horizontal > 180:
                relative_angle_horizontal -= 360 
      

        angle_to_cell_vertical = math.degrees(math.atan2(camera_h, flat_distance))  # Vertical angle
        angle_to_cell_vertical = angle_to_cell_vertical*-1 - cam_elevation_angle
    
        self.pixels = max(0,  1 - (flat_distance - min_distance +  half_square_width)/(max_range-(min_distance+half_square_width))) # max pixels = 100
        angle_to_closest_point_vertical = math.degrees(math.atan2(camera_h, flat_distance-half_square_width)) *-1 
        angle_to_farthest_point_vertical = math.degrees(math.atan2(camera_h, flat_distance+half_square_width)) *-1 
        if angle_to_farthest_point_vertical< cam_elevation_angle-v_fov/2 or angle_to_closest_point_vertical> cam_elevation_angle+v_fov/2:
            self.detection_probability = 0.0
            self.pixels = 0
            # print(f"Detection 0 for cell ({self.x}, {self.y}): Cell is outside vertical FOV. cam_elevation_angle-v_fov/2: {cam_elevation_angle-v_fov/2},cam_elevation_angle+v_fov/2: {cam_elevation_angle+v_fov/2}, angle_to_closest_point_vertical: {angle_to_closest_point_vertical}, angle_to_farthest_point_vertical: {angle_to_farthest_point_vertical}")
            return
        elif abs(angle_to_closest_point_vertical)>abs(cam_elevation_angle-v_fov/2):
            self.pixels = self.pixels * (1-(abs(angle_to_closest_point_vertical)-abs(cam_elevation_angle-v_fov/2))/(abs(angle_to_closest_point_vertical)-abs(angle_to_farthest_point_vertical)))
        
        # Determine the coordinates of the four corners of the grid cell

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
            relative_angle_corner = (angle_to_corner_deg - gaze_dir + 180) % 360
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
            self.pixels = 0
            # print(f"Detection 0 for cell ({self.x}, {self.y}): Cell is outside horizontal FOV. beta: {beta}, smallest angle: {leftmost_angle}, largest angle: {rightmost_angle}")
            return  # Cell is outside horizontal FOV
        elif max_angle > abs(beta):
            self.pixels = self.pixels * (1-(max_angle-beta)/(max_angle-min_angle))
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

                obs_left_angle = (math.degrees(math.atan2(obs_left_y - robot_state.y, obs_left_x - robot_state.x))+180 - gaze_dir) % 360
                if obs_left_angle > 180:
                    obs_left_angle -= 360  # Adjust to be between -180 and 180

                obs_right_angle = (math.degrees(math.atan2(obs_right_y - robot_state.y, obs_right_x - robot_state.x))+180 - gaze_dir) % 360
                if obs_right_angle > 180:
                    obs_right_angle -= 360  # Adjust to be between -180 and 180
                leftmost_obs_angle, rightmost_obs_angle = min(obs_left_angle, obs_right_angle), max(obs_left_angle, obs_right_angle)
                # print(f"({self.x}, {self.y}): rightmost_angle {rightmost_angle}, rightmost_obs_angle {rightmost_obs_angle},leftmost_angle {leftmost_angle}, leftmost_obs_angle {leftmost_obs_angle}")
                if leftmost_obs_angle <= leftmost_angle and rightmost_obs_angle>= rightmost_angle:
                    self.pixels = 0
                    self.detection_probability = 0
                    return
                elif leftmost_angle <= leftmost_obs_angle and leftmost_obs_angle<= rightmost_angle:
                    right_overlap = abs(leftmost_obs_angle-rightmost_angle)/abs(leftmost_angle-rightmost_angle)
                    max_right_overlap = max(right_overlap,max_right_overlap)
  
                elif leftmost_angle <= rightmost_obs_angle and rightmost_obs_angle <= rightmost_angle:
                    left_overlap = abs(leftmost_angle-rightmost_obs_angle)/abs(leftmost_angle-rightmost_angle)
                    max_left_overlap= max(max_left_overlap,left_overlap)
            
        self.pixels = self.pixels * (1-max_left_overlap) * (1-max_right_overlap)
        if self.pixels == 0:
            self.detection_probability = 0
        


        self.cx = (sensor_width / 2) * (1 + relative_angle_horizontal/ beta)
        self.cy = (sensor_height / 2) * (1 + math.radians(angle_to_cell_vertical) / math.radians(v_fov / 2))
        # print(f"({self.x}, {self.y}): self.detection_probability {self.detection_probability}, flat_distance {flat_distance}, pixels {self.pixels}, left overlap {max_left_overlap}, right overlap {max_right_overlap}")


   

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
        self.pixels = 0
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
        self.pixels = 0
        return 


# The environment and simulation class
class Environment:
    def __init__(self, task="RoboNav", starting_pos=(0, 0, 0, 0), target_pos=(1, 5), obstacles=[],star_position = 1):
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

        # Initialize obstacles
        # self.obstacles = [Obstacle(i * self.square_size + self.square_size / 2,
        #                            j * self.square_size + self.square_size / 2, height=5)
        #                   for i, j in obstacles]

        # List of all detectable objects
        self.detectable_objects = []
        self.detectable_objects.extend(self.grid.flatten())
        self.detectable_objects.extend(self.obstacles)
        self.detectable_objects.append(self.star)
        self.show_frames = True

      

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
                'time': self.sim_time,
                'x': obj.x,
                'y': obj.y,
                'type': obj.color_id,  # Type of object (GridCell, Obstacle)
                'detection_probability': obj.detection_probability,
                'pixels':  obj.pixels
            })

    def log_robot_state(self):
        """
        Log the robot's state (x, y, theta, phi).
        """
        self.robot_states.append({
            'time': self.sim_time,
            'x': self.robot.state.x,
            'y': self.robot.state.y,
            'heading': self.robot.state.theta,
            'gaze': self.robot.state.phi
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
            obj.update_detection_probability(robot_state, max_range, self.obstacles,self.robot.cam, bottom_fraction)

        # Log detection probabilities
        if hasattr(self, "max_sim_time"):
            self.log_detection_probabilities()
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

    def draw_view(self, robot_state):
        """
        Draw the current state of the environment including the robot's view and detection probabilities.
        """
        gaze_dir = robot_state.theta + robot_state.phi
        # Set up the plot
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.grid_size[0] * self.square_size)
        ax.set_ylim(0, self.grid_size[1] * self.square_size)
        ax.set_aspect('equal')
        ax.set_title('Robot Field of View')
        ax.set_xlabel('X Position (cm)')
        ax.set_ylabel('Y Position (cm)')

        # Iterate through the grid to draw grid cells and obstacles
        for i in range(self.grid_size[0]):
            for j in range(self.grid_size[1]):
                obj = self.grid[i][j]

                if obj.color_id == 1:  # Target (green)
                    if obj.detection_probability == 1:
                        color = 'green'
                    else:
                        color = plt.cm.Greens(obj.detection_probability)
                    edgecolor = 'green'
                elif obj.color_id == 2:  # Obstacle (red with shades based on detection probability)
                    if obj.detection_probability == 1:
                        color = 'red'
                    else:
                    # color = plt.cm.Reds(obj.detection_probability)
                        color = plt.cm.Reds(obj.detection_probability)
                    edgecolor = "red"
                else:  # Free space (blue)
                    color = plt.cm.Blues(obj.pixels)
                    
                    # print(f"color: {obj.detection_probability}")
                    edgecolor = "blue"

                # Draw a smaller square in the center of the grid for each object
                small_square_size = self.square_size / 3
                small_rect = patches.Rectangle(
                    (obj.x - small_square_size / 2, obj.y - small_square_size / 2),
                    small_square_size, small_square_size, linewidth=1, edgecolor=edgecolor, facecolor=color
                )
                ax.add_patch(small_rect)

        # Draw the robot as a rotated rectangle
        robot_length = 10  # Long side (in cm)
        robot_width = 8    # Short side (in cm)

        # Create the robot rectangle at the current robot position
        robot_rect = patches.Rectangle(
            (robot_state.x - robot_width / 2, robot_state.y - robot_length / 2),
            robot_width, robot_length,
            linewidth=1, edgecolor='black', facecolor='black'
        )

        # Apply a rotation transformation based on the robot's gaze direction
        t = mtransforms.Affine2D().rotate_deg_around(robot_state.x, robot_state.y, gaze_dir) + ax.transData
        robot_rect.set_transform(t)

        # Add the robot rectangle to the plot
        ax.add_patch(robot_rect)

        # Draw grid lines
        for x in range(0, (self.grid_size[0] + 1) * self.square_size, self.square_size):
            ax.axvline(x, color='black', linestyle='-', linewidth=1)
        for y in range(0, (self.grid_size[1] + 1) * self.square_size, self.square_size):
            ax.axhline(y, color='black', linestyle='-', linewidth=1)

        plt.gca().invert_yaxis()  # Invert Y-axis to match matrix indexing
        plt.colorbar(plt.cm.ScalarMappable(cmap='Blues'), ax=ax, orientation='vertical', label='Pixels # ')
        plt.show()

