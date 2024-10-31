import plotly.graph_objs as go
import plotly.offline as pyo
import numpy as np
from scipy.spatial.transform import Rotation as R

class TrajectoryPlanner:
    def __init__(self):
        self.trajectories = []  # List to store all trajectory traces
        self.num_traj = 0
        self.figure = None  # Placeholder for Plotly figure
        self.current_plot_filename = 'ur3e_trajectory.html'  # Filename for HTML output
        self.title = 'UR3e End Effector Trajectories'  # Default title for the plot
        self.description = 'Refresh to update the plot. Click on a trajectory in the legend to hide, or double click to isolate. Grey segments indicate end effector orientation and red markers highlight points with unsolved inverse kinematics.'  # Default description
        self.colors = colors = ["#1f77b4",  # blue
                                "#2ca02c",  # green
                                "#8c564b",  # brown
                                "#e377c2",  # pink
                                "#9467bd",  # purple
                                "#bcbd22",  # yellow-green
                                "#17becf",   # cyan
                                "#ff7f0e",  # orange
                                "#d62728",  # purple
                            ]  # Example colors for trajectories

    def add_trajectory(self, coordinates,joint_positions):
        """
        Add a trajectory to the planner.

        Args:
        coordinates (list): List of global coordinates. Used for 
                            highlighting end effector point.
        joint_positions (list): List of joint positions, used as an indicator of
                            failed inverse kinematics.
        """
        self.num_traj += 1
        
        # Extract x, y, z coordinates and orientations
        positions = [coord[:3] for coord in coordinates]
        euler_angles = [coord[3:] for coord in coordinates]

        x = [pos[0] for pos in positions]
        y = [pos[1] for pos in positions]
        z = [pos[2] for pos in positions]

        # Change marker color and size to large red if inverse kinematics failed.
        marker_colors = ['red' if joint is None else self.colors[self.num_traj % len(self.colors)] for joint in joint_positions]
        marker_size = [12 if joint is None else 6 for joint in joint_positions]

        # Create the trajectory trace
        trajectory_trace = go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode='lines+markers',
            marker=dict(size=marker_size, color=marker_colors, opacity=0.8),
            line=dict(color=self.colors[self.num_traj % len(self.colors)], width=2),
            name=f'Trajectory #{self.num_traj}'
        )

        # Append trajectory trace to list of trajectories
        self.trajectories.append(trajectory_trace)

        # Create the orientation vector trace
        orientation_traces = self._create_orientation_trace(positions, euler_angles)
        self.trajectories.extend(orientation_traces)  # Extend with orientation traces

        # Update the plot
        self.update_plot()

    def _create_orientation_trace(self, positions, euler_angles):
        """
        Create orientation vector traces.

        Args:
            positions: List of [x, y, z] positions.
            euler_angles: List of [rx, ry, rz] Euler angles (in radians).
        Return:
            return: List of orientation vector traces.
        """
        orientation_traces = []
        for i,(pos, euler) in enumerate(zip(positions, euler_angles)):
            # Convert Euler angles to rotation matrix using scipy
            r = R.from_euler('xyz', euler,degrees=True)
            rotation_matrix = r.as_matrix()

            # Define the length of the orientation vector
            vector_length = 0.05
            orientation_vector = np.dot(rotation_matrix, np.array([0, 0, -vector_length]))

            # Create a trace for the orientation vector
            if i == 0:
                first_entry = True
            else:
                first_entry = False
            orientation_traces.append(go.Scatter3d(
                x=[pos[0], pos[0] + orientation_vector[0]],
                y=[pos[1], pos[1] + orientation_vector[1]],
                z=[pos[2], pos[2] - orientation_vector[2]],
                mode='lines+markers',
                line=dict(color='grey', width=3),
                marker=dict(size=2, color='grey'),
                name=f'Trajectory #{self.num_traj} Orientation Vectors',  # All orientation vectors under one legend entry
                legendgroup=f'orientation{self.num_traj}',  # Group orientation vectors in legend
                showlegend=first_entry  # Set initial visibility based on flag
            ))

        return orientation_traces

    def update_plot(self):
        """
        Update the plot with current trajectories.
        """
        # Define the layout with fixed axis limits
        layout = go.Layout(
            title=dict(text=self.title, font=dict(size=20), automargin=True, yref='container'),
            scene=dict(
                xaxis=dict(title='X', range=[-0.5, 0.5]),
                yaxis=dict(title='Y', range=[-0.5, 0.5]),
                zaxis=dict(title='Z', range=[0, 1]),
                camera=dict(
                    eye=dict(x=1.0, y=-2.0, z=0.8)  # Specify initial camera position
                )
            ),
            margin=dict(l=0, r=0, b=0, t=0),
            legend=dict(x=0.8, y=0.8),  # Adjust legend position
            annotations=[
                dict(
                    text=self.description,
                    showarrow=False,
                    xref="paper",
                    yref="paper",
                    x=0,
                    y=-0.1
                )
            ],
        )
        
        # Combine all traces
        data = self.trajectories

        # Create a figure
        self.figure = go.Figure(data=data, layout=layout)

        # Display the plot in the notebook (optional)
        # pyo.iplot(self.figure)

        # Export the plot to an HTML file
        pyo.plot(self.figure, filename=self.current_plot_filename)


    def clear_plot(self):
        """
        Clear all trajectories from the plot.
        """
        self.trajectories = []
        self.update_plot()
