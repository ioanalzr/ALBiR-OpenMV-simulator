# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
# import pandas as pd
# import numpy as np
from environment import Environment
from dataclasses import dataclass
from typing import Tuple, List
import logging


@dataclass
class TestCase:
    """Data class to hold test configuration"""
    obstacles: List[Tuple[int, int]]
    starting_pos: Tuple[int, int, int, int]
    target_pos: Tuple[int, int]
    speed: float = 0.1
    bias: float = 0
    max_sim_time: float = 50 # s
    time_step: float = 0.01 # s
    task: str = "RoboNav"
    log_level: int = logging.INFO
    detection_factor: float = 1.0 # max = 1
    servo_error: float = 0 # (0/1/2/3: servo_noise sd will be 0.001*servo_error)
    pan_error: float = 0 # sd of pan angle in deg
    frame_buffer: bool = False
    pan_delay: float = 0 # time it takes for the mirror to move 1 deg
    speed_coef: float = 2 # can be used to tune speed of the robot - speed = speed * speed_coef

def run_test(test_case: TestCase):
    """Run a single test case and plot results"""
    env = Environment(
        task=test_case.task,
        starting_pos=test_case.starting_pos,
        target_pos=test_case.target_pos,
        obstacles=test_case.obstacles,
        log_level=test_case.log_level,
        detection_factor = test_case.detection_factor,
        servo_error = test_case.servo_error,
        pan_error = test_case.pan_error,
        pan_delay = test_case.pan_delay
    
    )
    
    try:
        # Run the simulation
        env.run_simulation(
            speed=test_case.speed,
            bias=test_case.bias,
            max_sim_time=test_case.max_sim_time,
            time_step=test_case.time_step,
            task=test_case.task,
            frame_buffer=test_case.frame_buffer
            
        )
    except TimeoutError as e:
        print(f"Simulation terminated: {e}")
    finally:
        # Ensure animate_trajectory runs even if run_simulation fails
        env.visualizer.animate_trajectory(test_case.obstacles, 'robot_states.csv', frame_skip=10)

    return env

# Define test cases
TEST_CASES = {
    "complex_obstacle": TestCase(
        obstacles=[(1, 1), (0, 3), (2, 1), (5, 2), (2, 3), (1,3), (1,4), (3,1)],
        # obstacles=[(1, 1), (2, 1),(5, 2), (2, 3), (1,3), (1,4), (3,1)],
        starting_pos=(0,0,0,0),
        target_pos=(3,5),
        log_level=logging.INFO,
        detection_factor = 1, 
        servo_error = 0, 
        pan_error = 0
    ),
    "simple_obstacle": TestCase(
        obstacles=[(1, 2), (0, 3), (2, 1), (1, 4)],
        # obstacles=[(1, 2), (2, 1), (1, 4)],
        starting_pos=(0,0,0,0),
        target_pos=(0,5),
        log_level=logging.INFO,
        detection_factor = 1,
        servo_error = 0, 
        pan_error = 0
    ),
    "offset_start": TestCase(
        obstacles=[(1, 2), (0, 3), (2, 1)],
        # obstacles=[ (0, 3), (2, 1)],
        starting_pos=(1,0,0,0),
        target_pos=(0,5),
        log_level=logging.INFO,
        detection_factor = 1,
        servo_error = 0, 
        pan_error = 0
    ),
    "L": TestCase(
        obstacles=[(2, 1), (2, 2), (2, 3), (2, 4)],
        # obstacles=[ (0, 3), (2, 1)],
        starting_pos=(3,0,0,0),
        target_pos=(0,5),
        log_level=logging.INFO,
        detection_factor = 1,
        servo_error = 0,
        pan_error = 0
    )
}

if __name__ == "__main__":
    # Run specific test case
    # test_case = TEST_CASES["complex_obstacle"]
    # run_test(test_case)
    
    # Or run all test cases
    for name, test_case in TEST_CASES.items():
        print(f"\nRunning test case: {name}")
        run_test(test_case)
