# ALBiR-OpenMV-teaching-platform

The robotsim environment helps testing the code for operating the OpenMV differential drive robot used as a teaching platform in the maze assignment of the MEng module Animal Locomotion and Bioinspired. 

The functions defined in robot.py to drive the robot can be plugged into robot_sim.py (follow instructions at the top of the script). The modules that drive the physical servos and interact with the camera module of the robot (servos.py and camera.py) are replaced by modules which interact with a simulated environment, updating the state of the robot in the environment (servos_sim.py) and estimating its view (size, colour, and detection probability) of the maze objects based on geometry (camera_sim.py). 

The simulator currently supports the RoboNav task: a 4 × 6 grid of 18 cm squares, where blue cells are free path, red cells are obstacles and one green cell is the target (see environment.py).

Each test case opens an interactive animation in your browser.


## 1. Setting up the environment
```bash
git clone https://github.com/ioanalzr/ALBiR-OpenMV-simulator.git
cd ALBiR-OpenMV-simulator

conda create -n robotsim python=3.10
conda activate robotsim
pip install -r requirements.txt
```
## 2. Running the test cases set in test_simulation.py
```bash
cd simulation
python test_simulation.py
```

## Checking the robot's view and tuning parameters

In test_view.ipynb, you can visualise the observed size (a.u.) of the grid objects given the current state of the robot.

```python
from environment import *

env = Environment(task="RoboNav",
                  starting_pos=(0, 0, 0, 0),
                  target_pos=(0, 5),
                  obstacles=[(2, 1), (2, 2), (2, 3), (2, 4)],
                  log_level=logging.INFO)

# State(x_cm, y_cm, heading_deg, pan_deg, l_speed=0, r_speed=0)
robot_state = State(45, 9, 20, 0, 0)
env.visualizer.test_view(robot_state)
```

Sensor properties can be changed in camera_sim.py.




