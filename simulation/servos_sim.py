import random 
import math
import logging

logger = logging.getLogger(__name__)

class State:
    def __init__(self, x: float, y: float, theta: float, phi = float, l_speed = 0, r_speed = 0):
        """
        Initialize the state of the robot.

        Args:
            x (float): The x position of the robot (in cm).
            y (float): The y position of the robot (in cm).
            theta (float): The orientation of the robot (in degrees)
            phi (float): pan position (in degrees)
            gaze direction is theta+phi
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.phi = phi
        self.l_speed = l_speed
        self.r_speed = r_speed
    


    def update(self, delta_x: float, delta_y: float, delta_theta: float, delta_phi, curr_l_speed = None, curr_r_speed = None):
        """
        Update the state of the robot.

        Args:
            delta_x (float): Change in the x position (in cm).
            delta_y (float): Change in the y position (in cm).
            delta_theta (float): Change in orientation (in radians).
        """
        self.x += delta_x
        self.y += delta_y
        self.theta = self.theta + delta_theta
        self.phi+= delta_phi # self.servo.pan_pos
        if curr_l_speed is not None:
            self.l_speed = curr_l_speed
        if curr_r_speed is not None:
            self.r_speed = curr_r_speed
        logger.debug(f"State updated - x: {self.x}, y: {self.y}, theta: {self.theta}, phi: {self.phi}")
        

    def get_position(self):
        """
        Get the current position of the robot.

        Returns:
            tuple: The (x, y) position of the robot.
        """
        return self.x, self.y

    def get_orientation(self):
        """
        Get the current orientation of the robot.

        Returns:
            float: The orientation (theta) of the robot in radians.
        """
        return self.theta
    
    def get_gaze(self):
        """
        Get the current orientation of the robot.

        Returns:
            float: The orientation (theta) of the robot in radians.
        """
        return self.phi


    def get_speed(self):
        """
        Get the current speed of each wheel

        Returns:
            float: The orientation (theta) of the robot in radians.
        """
        return self.l_speed, self.r_speed
    

class Servo:
    """
    A class responsible for controlling servos via the PCA9685 controller.
    """

    def __init__(self, robot, servo_error = 0, pan_error = 0):
        """
        Initialise the servo object and sets the tuning coefficients.
        """
        self.robot = robot
        # self.robot.state = State(self.robot.cam.env.starting_pos[0] * self.robot.cam.env.square_size + self.robot.cam.env.square_size / 2,
        #                    self.robot.cam.env.starting_pos[1] * self.robot.cam.env.square_size + self.robot.cam.env.square_size / 2, 0)

        # Servo tuning coefficients

        self.left_zero = 0.1
        self.right_zero = -0.1

        # Set up servo angle limits
        self.degrees = 90
        self.min_deg = -self.degrees / 2
        self.max_deg = self.degrees / 2

        # self.curr_l_speed = 0
        # self.curr_r_speed = 0
        self.pan_pos = 0

        self.noise_std = servo_error*0.001
        self.pan_error_std = pan_error # deg
        self.pan_delay = 0 #s

        self.max_speed = 2
        
        # self.freq = 50
        # self.period = 1000000 / self.freq

        # # Calculate duty cycles for PWM signal range
        # self.min_duty = self._us2duty(700)
        # self.max_duty = self._us2duty(2300)
        # self.mid_duty = (self.min_duty + self.max_duty) / 2
        # self.span = (self.max_duty - self.min_duty)

        # # Initialise the PCA9685 controller for I2C communication
        # self.pca9685 = pca9685.PCA9685(SoftI2C(sda=Pin('P5'), scl=Pin('P4')), 0x40)
        # self.pca9685.freq(self.freq)

        self.left_id = 4
        self.right_id = 5
        self.pan_id = 1


    def set_differential_drive(self, speed: float, bias: float) -> None:
        bias = bias*-1
        coeff = 1
        speed = speed* coeff
        speed = max(min(speed, 10), 0)
        bias = max(min(bias, 10), -1)
        left_speed = speed * (1 - bias)
        right_speed = speed * (1 + bias)
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 1:
            left_speed /= max_speed
            right_speed /= max_speed
        
        self.set_speed(left_speed, right_speed)
        
 
    def sleep_ms_simulated(self, ms: int) -> None:
        total_time = ms / 1000.0
        elapsed_time = 0

        while elapsed_time < total_time:
            self.robot.cam.env.update_time()
            elapsed_time += self.robot.cam.env.delta_t
            # if self.robot.cam.env.sim_time >= self.robot.cam.env.max_sim_time:
            #     # log data
            #     self.robot.cam.env.save_to_csv()
            #     raise TimeoutError("Simulation time exceeded")
        return

    def sleep_ms_simulated_old(self, ms: int) -> None:
        total_time = ms / 1000.0
        elapsed_time = 0

        while elapsed_time < total_time:
            logger.debug(f"Simulation time elapsed: {elapsed_time}")
            # self.set_speed(self.curr_l_speed, self.curr_r_speed,self.robot.cam.env.delta_t)
            v_right,v_left = self.robot.state.get_speed()
            v = (v_left + v_right) / 2  # Linear velocity
            omega = (v_right - v_left) / self.robot.WHEEL_BASE  # Angular velocity

            # Update position and orientation
            delta_theta = math.degrees(omega * delta_t)
            
            delta_x = -v * math.sin(math.radians(self.robot.state.get_orientation())) * delta_t * 100  # Convert to cm
            delta_y = v * math.cos(math.radians(self.robot.state.get_orientation())) * delta_t * 100 

            # Update the robot's state (phi remains fixed unless changed externally)
            self.robot.state.update(delta_x, delta_y, delta_theta,0,v_left,v_right) ####
            self.robot.cam.env.log_robot_state()
  
            elapsed_time += self.robot.cam.env.delta_t
            self.robot.cam.env.sim_time += self.robot.cam.env.delta_t
            if self.robot.cam.env.sim_time >= self.robot.cam.env.max_sim_time:
                # log data
                self.robot.cam.env.save_to_csv()
                raise TimeoutError("Simulation time exceeded")
        return

    def set_spin(self, speed: float) -> None:
        """
        Set speeds for a differential drive robot to spin in place.

        Args:
            speed (float): Speed coefficient of the robot (0 to 1).
        """
        # Validate input ranges
        speed = max(min(speed, 10), 0)

        # Calculate individual wheel speeds
        left_speed = -speed
        right_speed = speed

        # Normalize speeds if they exceed 1
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 1:
            left_speed /= max_speed
            right_speed /= max_speed

        self.set_speed(left_speed, right_speed)
    

    def set_angle(self, angle: float) -> float:
        """
        Update the pan angle of the robot.

        Args:
            angle (float): Desired angle (in degrees) for the pan servo.

        Returns:
            float: The updated pan angle (in degrees).
        """
        # Constrain angle to the limits
        angle = max(min(angle, self.max_deg), self.min_deg) # the recorded angle
        delta_angle = angle - self.robot.state.get_gaze()
        # if abs(delta_angle)<1:
        #      delta_angle_noisy = 0
        # else:
        delta_angle_noisy = random.gauss(delta_angle, self.pan_error_std) # actual angle is different 
        # Update the pan angle
        # add noise to pan
        # delta_angle_noisy = delta_angle
        ## self.robot.state.update(0,0,0,delta_angle_noisy) 
        ## self.pan_pos = self.robot.state.get_gaze()  ### what is this for - bcs servo.pan_pos was in the initial code
        self.pan_pos= angle
        self.robot.cam.env.update_time(self.pan_delay*abs(delta_angle_noisy))
        self.robot.state.update(0,0,0,delta_angle_noisy)
        

        return angle # return recorded angle not actual
    def calculate_noise(self,noise_std,speed):
                # Avoid division by zero, and set higher noise at slower speeds
                return noise_std / max(abs(speed), 0.1)  # Prevent division by zero with max()

    def set_speed(self, l_speed: float, r_speed: float) -> None:
    
        max_speed = self.max_speed      # Max wheel speed (normalized)
        min_speed = -self.max_speed   # Min wheel speed (negative for reverse)

       
        # Calculate noise bounds based on speed
        noise_bound_l = self.calculate_noise(self.noise_std, l_speed)
        noise_bound_r = self.calculate_noise(self.noise_std, r_speed)

        # Apply Gaussian noise to the wheel speeds
        l_speed_noisy = random.gauss(l_speed, noise_bound_l)
        r_speed_noisy = random.gauss(r_speed, noise_bound_r)

        # Ensure the noisy speeds are within bounds
        l_speed_noisy = max(min(l_speed_noisy, max_speed), min_speed)
        r_speed_noisy = max(min(r_speed_noisy, max_speed), min_speed)

        # Use the noisy speeds for movement calculation
        self.robot.state.update(0,0,0,0,l_speed_noisy,r_speed_noisy ) 
        
        



    def _duty2us(self, value: float) -> int:
        """
        Convert a given PWM duty cycle value to microseconds.

        Args:
            value (float): PWM duty cycle value.

        Returns:
            int: Corresponding value in microseconds.
        """
        return int(value * self.period / 4095)

    def _us2duty(self, value: float) -> int:
        """
        Convert a given value in microseconds to PWM duty cycle.

        Args:
            value (float): Value in microseconds.

        Returns:
            int: Corresponding PWM duty cycle value.
        """
        return int(4095 * value / self.period)

    def release(self, idx: int) -> None:
        """
        Simple servo release method.

        Args:
            idx (int): Servo shield pin ID to reset.
        """
        self.pca9685.duty(idx, 0)

    def soft_reset(self) -> None:
        """
        Method to reset the servos to default and print a delay prompt.
        """
        # Reset all servo shield pins
        for i in range(0, 7, 1):
            self.pca9685.duty(i, 0)

        # Reset pan to center
        self.set_angle(0)

        # Print delay prompt
        for i in range(3, 0, -1):
            print(f"{i} seconds remaining.")
            self.servo.sleep_ms_simulated(1000) 

        print("___Running Code___")
