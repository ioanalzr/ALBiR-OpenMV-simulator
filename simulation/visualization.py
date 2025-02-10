import plotly.express as px
import pandas as pd
# import numpy as np
import logging
import plotly.graph_objects as go
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.transforms import Affine2D
import matplotlib.cm as cm  # For color maps like Blues, Reds, Greens
import matplotlib.colorbar as cbar  # For adding color bar if necessary

logger = logging.getLogger(__name__)

class EnvironmentVisualizer:
    def __init__(self, environment):
        self.env = environment

    def plot_trajectory(self, obstacles_pos, states_file,snapshots_file = "snapshots.csv"):
        """
        Plot the robot's trajectory using Plotly Express
        """
        logger.info(f"Starting trajectory plot with {len(obstacles_pos)} obstacles")
        
        try:
            # Read robot states
            logger.debug(f"Reading robot states from {states_file}")
            robot_states_df = pd.read_csv(states_file)
            logger.debug(f"Found {len(robot_states_df)} robot state entries")
            snapshots_df = pd.read_csv(snapshots_file)
            
            # Create environment grid data
            grid_data = []
            for i in range(self.env.grid_size[0]):
                for j in range(self.env.grid_size[1]):
                    x = i * self.env.square_size + self.env.square_size / 2
                    y = j * self.env.square_size + self.env.square_size / 2
                    # Determine cell type
                    if (i, j) in obstacles_pos:
                        cell_type = "Obstacle"
                    elif (i, j) == self.env.target:
                        cell_type = "Target"
                    else:
                        cell_type = "Free Space"
                    
                    grid_data.append({
                        'x': x,
                        'y': y,
                        'type': cell_type
                    })
            
            grid_df = pd.DataFrame(grid_data)
            
            # Create scatter plot with square markers
            fig = px.scatter(grid_df, x='x', y='y', color='type',
                           color_discrete_map={
                               'Obstacle': 'red',
                               'Target': 'green',
                               'Free Space': 'blue'
                           },
                           title="Robot Simulation Result",
                           width=600,
                           height=718)
            
            # Update grid markers to be squares and larger
            fig.update_traces(marker=dict(symbol="square", size=24), selector=dict(mode='markers'))

            # Add grid lines within board dimensions
            board_width = self.env.grid_size[0] * self.env.square_size
            board_height = self.env.grid_size[1] * self.env.square_size
            
            # Vertical lines
            for i in range(self.env.grid_size[0] + 1):
                x = i * self.env.square_size
                fig.add_shape(
                    type="line",
                    x0=x,
                    y0=0,
                    x1=x,
                    y1=board_height,
                    line=dict(width=1)
                )
            
            # Horizontal lines
            for i in range(self.env.grid_size[1] + 1):
                y = i * self.env.square_size
                fig.add_shape(
                    type="line",
                    x0=0,
                    y0=y,
                    x1=board_width,
                    y1=y,
                    line=dict(width=1)
                )
            
            # Add robot trajectory
            fig.add_scatter(x=robot_states_df['x'], y=robot_states_df['y'],
                          mode='lines+markers',
                          name='Robot Trajectory',
                          line=dict(width=2),
                          marker=dict(color='black', size=8))
            
            # Add final position
            fig.add_scatter(x=[robot_states_df.iloc[-1]['x']],
                          y=[robot_states_df.iloc[-1]['y']],
                          mode='markers',
                          name='Final Position',
                          marker=dict(color='black', size=16, symbol='star'))
            
            # Update layout
            fig.update_layout(
                xaxis_title="X Position (cm)",
                yaxis_title="Y Position (cm)",
                xaxis=dict(
                    range=[0, self.env.grid_size[0] * self.env.square_size],
                    tickmode='array',
                    tickvals=[i * self.env.square_size for i in range(self.env.grid_size[0] + 1)]
                ),
                yaxis=dict(
                    range=[self.env.grid_size[1] * self.env.square_size, 0],
                    tickmode='array',
                    tickvals=[i * self.env.square_size for i in range(self.env.grid_size[1] + 1)],
                    scaleanchor="x",
                    scaleratio=1
                )
            )
        
            # Plotly dark theme
            # fig.update_layout(template='plotly_dark')

            logger.info("Trajectory plot completed successfully")
            fig.show()
            
        except Exception as e:
            logger.error(f"Error creating trajectory plot: {str(e)}")
            raise
    
    def animate_trajectory(self, obstacles_pos, states_file, frame_skip=1):
        """
        Create an animated visualization of the robot's trajectory
        """
        logger.info(f"Starting trajectory animation with frame_skip={frame_skip}")
        
        try:
            # Read robot states
            robot_states_df = pd.read_csv(states_file)
            robot_states_df = robot_states_df[::frame_skip].copy()  # Apply frame skip
            
            # Create environment grid data
            grid_data = []
            for i in range(self.env.grid_size[0]):
                for j in range(self.env.grid_size[1]):
                    x = i * self.env.square_size + self.env.square_size / 2
                    y = j * self.env.square_size + self.env.square_size / 2
                    
                    if (i, j) in obstacles_pos:
                        cell_type = "Obstacle"
                    elif (i, j) == self.env.target:
                        cell_type = "Target"
                    else:
                        cell_type = "Free Space"
                    
                    grid_data.append({
                        'x': x,
                        'y': y,
                        'type': cell_type
                    })
            
            grid_df = pd.DataFrame(grid_data)
            
            # Create base figure with static grid
            fig = go.Figure()
            
            # Add grid scatter plot
            for cell_type in ["Free Space", "Obstacle", "Target"]:
                mask = grid_df['type'] == cell_type
                fig.add_trace(go.Scatter(
                    x=grid_df[mask]['x'],
                    y=grid_df[mask]['y'],
                    mode='markers',
                    name=cell_type,
                    marker=dict(
                        symbol="square",
                        size=24,
                        color={'Free Space': 'blue', 'Obstacle': 'red', 'Target': 'green'}[cell_type]
                    )
                ))
            
            # Add initial robot position and trajectory line
            fig.add_trace(go.Scatter(
                x=[robot_states_df.iloc[0]['x']],
                y=[robot_states_df.iloc[0]['y']],
                mode='markers',
                name='Robot',
                marker=dict(
                    color='black',
                    size=40,
                    # symbol='triangle-up',  # Make robot a triangle
                    symbol='square',
                    angle=robot_states_df.iloc[0]['heading']  # Set initial rotation
                )
            ))

            fig.add_trace(go.Scatter(
                x=[robot_states_df.iloc[0]['x']],
                y=[robot_states_df.iloc[0]['y']],
                mode='markers',
                name='Robot gaze',
                marker=dict(
                    color='white',
                    size=10,
                    symbol='triangle-up',  # Make robot a triangle
                    angle=robot_states_df.iloc[0]['heading']+robot_states_df.iloc[0]['gaze']  # Set initial rotation
                )
            ))
            
            # Add trajectory trace (will be updated in frames)
            fig.add_trace(go.Scatter(
                x=[robot_states_df.iloc[0]['x']],
                y=[robot_states_df.iloc[0]['y']],
                mode='lines',
                name='Trajectory',
                line=dict(color='black', width=6)
            ))
            
            # Add grid lines
            board_width = self.env.grid_size[0] * self.env.square_size
            board_height = self.env.grid_size[1] * self.env.square_size
            
            for i in range(self.env.grid_size[0] + 1):
                x = i * self.env.square_size
                fig.add_shape(type="line", x0=x, y0=0, x1=x, y1=board_height, line=dict(width=1))
            
            for i in range(self.env.grid_size[1] + 1):
                y = i * self.env.square_size
                fig.add_shape(type="line", x0=0, y0=y, x1=board_width, y1=y, line=dict(width=1))
            
            # Create frames for robot movement
            frames = []
            times = robot_states_df['time'].unique()
            
            for i, t in enumerate(times):
                frame_data = robot_states_df[robot_states_df['time'] <= t]
                current_pos = frame_data.iloc[-1]
                
                frame = go.Frame(
                    data=[
                        *fig.data[:-3],  # Keep grid traces
                        # Update trajectory
                        go.Scatter(
                            x=frame_data['x'],
                            y=frame_data['y'],
                            mode='lines',
                            name='Trajectory',
                            line=dict(color='black', width=6)
                        ),
                         # Update robot position
                        go.Scatter(
                            x=[current_pos['x']],
                            y=[current_pos['y']],
                            mode='markers',
                            name='Robot',
                            marker=dict(
                                color='black',
                                size=40,
                                # symbol='triangle-up',
                                symbol='square',
                                angle=current_pos['heading']
                                # +current_pos['gaze']  # Update rotation
                            )
                        ),
                        go.Scatter(
                            x=[current_pos['x']],
                            y=[current_pos['y']],
                            mode='markers',
                            name='Robot gaze',
                            marker=dict(
                                color='white',
                                size=10,
                                symbol='triangle-up',
                                angle=current_pos['heading']+current_pos['gaze']
                                # +current_pos['gaze']  # Update rotation
                            )
                        )
                       
                    ],
                    name=f"frame_{t}"
                )
                frames.append(frame)
            
            fig.frames = frames
            
            # Add slider and play button side by side
            updatemenus = [{
                'type': 'buttons',
                'showactive': False,
                'x': 0.1,
                'y': -0.1,  # Move buttons below plot
                'xanchor': 'right',
                'yanchor': 'top',
                'pad': {'t': 0, 'r': 10},
                'buttons': [
                    {
                        'label': '▶️ Play',
                        'method': 'animate',
                        'args': [None, {
                            'frame': {'duration': 50, 'redraw': True},
                            'fromcurrent': True,
                            'transition': {'duration': 0}
                        }]
                    },
                    {
                        'label': '⏸️ Pause',
                        'method': 'animate',
                        'args': [[None], {
                            'frame': {'duration': 0, 'redraw': False},
                            'mode': 'immediate',
                            'transition': {'duration': 0}
                        }]
                    }
                ]
            }]
            
            # Configure slider
            sliders = [{
                'currentvalue': {"prefix": "Time: "},
                'pad': {"t": 0},  # Reduce padding
                'len': 0.9,  # Make slider longer
                'x': 0.1,  # Align with buttons
                'y': -0.1,  # Position below plot
                'xanchor': 'left',
                'yanchor': 'top',
                'steps': [
                    {
                        'args': [
                            [f"frame_{t}"],
                            {
                                "frame": {"duration": 0, "redraw": True},
                                "mode": "immediate",
                            }
                        ],
                        'label': f"{t:.2f}",
                        'method': "animate"
                    }
                    for t in times
                ]
            }]
            
            # Update layout
            fig.update_layout(
                width=600,
                height=815,
                title="Robot Trajectory Animation",  # Added title
                xaxis_title="X Position (cm)",  # Added x-axis title
                yaxis_title="Y Position (cm)",  # Added y-axis title
                updatemenus=updatemenus,
                sliders=sliders,
                xaxis=dict(
                    range=[0, self.env.grid_size[0] * self.env.square_size],
                    tickmode='array',
                    tickvals=[i * self.env.square_size for i in range(self.env.grid_size[0] + 1)]
                ),
                yaxis=dict(
                    range=[self.env.grid_size[1] * self.env.square_size, 0],
                    tickmode='array',
                    tickvals=[i * self.env.square_size for i in range(self.env.grid_size[1] + 1)],
                    scaleanchor="x",
                    scaleratio=1
                ),
                margin=dict(t=100, b=100)  # Add margin for controls
            )
            
            logger.info("Animation created successfully")
            fig.show()
            
        except Exception as e:
            logger.error(f"Error creating animation: {str(e)}")
            raise




    def test_view(self, robot_state, max_range = 46):
        """
        Draw the current state of the environment including the robot's view and detection probabilities.
        """
        logger.debug("Starting test_view visualization.")

        gaze_dir = robot_state.theta + robot_state.phi

        # Set up the plot
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.env.grid_size[0] * self.env.square_size)
        ax.set_ylim(0, self.env.grid_size[1] * self.env.square_size)
        ax.set_aspect('equal')
        ax.set_title('Robot Field of View')
        ax.set_xlabel('X Position (cm)')
        ax.set_ylabel('Y Position (cm)')

        # Iterate through the grid to draw grid cells and obstacles
        for i in range(self.env.grid_size[0]):
            for j in range(self.env.grid_size[1]):
                obj = self.env.grid[i][j]
                obj.update_detection_probability(robot_state, max_range, self.env.obstacles, self.env.robot.cam, detection_factor = self.env.detection_factor,max_speed = self.env.robot.servo.max_speed)

                # Determine the color and edge based on detection probability or type
                if obj.color_id == 1:  # Target (green)
                    color = plt.cm.Greens(obj.detection_probability)
                    edgecolor = 'green'
                elif obj.color_id == 2:  # Obstacle (red)
                    color = plt.cm.Reds(obj.detection_probability)
                    edgecolor = "red"
                else:  # Free space (blue)
                    color = plt.cm.Blues(obj.detection_probability)
                    edgecolor = "blue"

                # Draw a smaller square in the center of the grid for each object
                small_square_size = self.env.square_size / 4
                small_rect = patches.Rectangle(
                    (obj.x - small_square_size / 2, obj.y - small_square_size / 2),
                    small_square_size, small_square_size, linewidth=1, edgecolor=edgecolor, facecolor=color
                )
                ax.add_patch(small_rect)

                # Add the number of pixels on top of each grid cell
                ax.text(
                    obj.x, obj.y, f"{int(obj._pixels)}",  # Display pixels as integer
                    # obj.x, obj.y, f"{int(obj._cx)}",
                    ha='center', va='center', fontsize=8, color='black', zorder=10
                )

        # # Draw the robot as a triangle (mimicking Plotly triangle-up marker)
        # robot_length = 10  # Long side (in cm)
        # robot_width = 8    # Short side (in cm)
        # robot_triangle = patches.Polygon(
        #     [
        #         (robot_state.x, robot_state.y - robot_length / 2),  # Top point
        #         (robot_state.x - robot_width / 2, robot_state.y + robot_length / 2),  # Bottom left
        #         (robot_state.x + robot_width / 2, robot_state.y + robot_length / 2)   # Bottom right
        #     ],
        #     closed=True,
        #     edgecolor='black',
        #     facecolor='black'
        # )
        # t = Affine2D().rotate_deg_around(robot_state.x, robot_state.y, gaze_dir) + ax.transData
        # robot_triangle.set_transform(t)
        # ax.add_patch(robot_triangle)
        robot_length = 8  # Long side (in cm)
        robot_width  = 7   # Short side (in cm)

        # Compute the lower-left corner so that the rectangle is centered at (robot_state.x, robot_state.y)
        rect_x = robot_state.x - robot_width / 2
        rect_y = robot_state.y - robot_length / 2

        robot_rect = patches.Rectangle(
            (rect_x, rect_y),
            robot_width,
            robot_length,
            linewidth=1,
            edgecolor='black',
            facecolor='black'
        )

        
        t_rect = Affine2D().rotate_deg_around(robot_state.x, robot_state.y, robot_state.theta) + ax.transData
        robot_rect.set_transform(t_rect)
        ax.add_patch(robot_rect)

        # Draw robot gaze marker as a small triangle

        gaze_size = 2
    
        triangle_coords = [
        (robot_state.x, robot_state.y - gaze_size / 2),  
        (robot_state.x - gaze_size / 2, robot_state.y + gaze_size / 2),  
        (robot_state.x + gaze_size / 2, robot_state.y + gaze_size / 2)   
    ]

        robot_gaze_triangle = patches.Polygon(
            triangle_coords,
            closed=True,
            edgecolor='white',
            facecolor='white'
        )
        
        t_triangle = Affine2D().rotate_deg_around(robot_state.x, robot_state.y, robot_state.phi+robot_state.theta) + ax.transData
        robot_gaze_triangle.set_transform(t_triangle)
        ax.add_patch(robot_gaze_triangle)
        # Draw grid lines
        for x in range(0, (self.env.grid_size[0] + 1) * self.env.square_size, self.env.square_size):
            ax.axvline(x, color='black', linestyle='-', linewidth=1)
        for y in range(0, (self.env.grid_size[1] + 1) * self.env.square_size, self.env.square_size):
            ax.axhline(y, color='black', linestyle='-', linewidth=1)

        # Add color bar for detection probability
        sm = plt.cm.ScalarMappable(cmap='Blues', norm=plt.Normalize(vmin=0, vmax=1))
        sm.set_array([])
        plt.colorbar(sm, ax=ax, orientation='vertical', label='Detection Probability')

        # Invert Y-axis to match matrix indexing
        plt.gca().invert_yaxis()

        logger.debug("Finished test_view visualization.")
        plt.show()

        
