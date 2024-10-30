import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd
import numpy as np
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
    speed: float = 0.2
    bias: float = 0
    max_sim_time: float = 100
    time_step: float = 0.01
    show_frames: bool = False
    task: str = "RoboNav"
    log_level: int = logging.INFO

def run_test(test_case: TestCase):
    """Run a single test case and plot results"""
    env = Environment(
        task=test_case.task,
        starting_pos=test_case.starting_pos,
        target_pos=test_case.target_pos,
        obstacles=test_case.obstacles,
        log_level=test_case.log_level
    )
    
    env.run_simulation(
        speed=test_case.speed,
        bias=test_case.bias,
        max_sim_time=test_case.max_sim_time,
        time_step=test_case.time_step,
        task=test_case.task,
        show_frames=test_case.show_frames
    )
    
    # Fix the argument order to match the method signature
    # env.visualizer.plot_trajectory(test_case.obstacles, 'robot_states.csv')
    
    # Uncomment to also show animation
    env.visualizer.animate_trajectory(test_case.obstacles, 'robot_states.csv', frame_skip=10)
    return env

# Define test cases
TEST_CASES = {
    "complex_obstacle": TestCase(
        obstacles=[(1, 1), (0, 3), (2, 1), (5, 2), (2, 3), (1,3), (1,4), (3,1)],
        starting_pos=(0,0,0,0),
        target_pos=(2,5),
        log_level=logging.ERROR
    ),
    "simple_obstacle": TestCase(
        obstacles=[(1, 2), (0, 3), (2, 1), (1, 4)],
        starting_pos=(0,0,0,0),
        target_pos=(0,5)
    ),
    "offset_start": TestCase(
        obstacles=[(1, 2), (0, 3), (2, 1)],
        starting_pos=(1,0,0,0),
        target_pos=(0,5)
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
