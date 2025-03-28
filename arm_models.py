from math import sin, cos, atan, acos, asin, sqrt, atan2
import numpy as np
from matplotlib.figure import Figure
from helper_fcns.utils import (
    EndEffector,
    rotm_to_euler,
    euler_to_rotm,
    dh_to_matrix,
    wraptopi,
    check_joint_limits,
)

PI = 3.1415926535897932384
np.set_printoptions(precision=3)
K = [0, 0, 1]
K2 = [1, 0, 0]


class Robot:
    """
    Represents a robot manipulator with various kinematic configurations.
    Provides methods to calculate forward kinematics, inverse kinematics, and velocity kinematics.
    Also includes methods to visualize the robot's motion and state in 3D.

    Attributes:
        num_joints (int): Number of joints in the robot.
        ee_coordinates (list): List of end-effector coordinates.
        robot (object): The robot object (e.g., TwoDOFRobot, ScaraRobot, etc.).
        origin (list): Origin of the coordinate system.
        axes_length (float): Length of the axes for visualization.
        point_x, point_y, point_z (list): Lists to store coordinates of points for visualization.
        show_animation (bool): Whether to show the animation or not.
        plot_limits (list): Limits for the plot view.
        fig (matplotlib.figure.Figure): Matplotlib figure for 3D visualization.
        sub1 (matplotlib.axes._subplots.Axes3DSubplot): Matplotlib 3D subplot.
    """

    def __init__(self, type="2-dof", show_animation: bool = True):
        """
        Initializes a robot with a specific configuration based on the type.

        Args:
            type (str, optional): Type of robot (e.g., '2-dof', 'scara', '5-dof'). Defaults to '2-dof'.
            show_animation (bool, optional): Whether to show animation of robot movement. Defaults to True.
        """
        if type == "2-dof":
            self.num_joints = 2
            self.ee_coordinates = ["X", "Y"]
            self.robot = TwoDOFRobot()

        elif type == "scara":
            self.num_joints = 3
            self.ee_coordinates = ["X", "Y", "Z", "Theta"]
            self.robot = ScaraRobot()

        elif type == "5-dof":
            self.num_joints = 5
            self.ee_coordinates = ["X", "Y", "Z", "RotX", "RotY", "RotZ"]
            self.robot = FiveDOFRobot()

        self.origin = [0.0, 0.0, 0.0]
        self.axes_length = 0.04
        self.point_x, self.point_y, self.point_z = [], [], []
        self.waypoint_x, self.waypoint_y, self.waypoint_z = [], [], []
        self.waypoint_rotx, self.waypoint_roty, self.waypoint_rotz = [], [], []
        self.show_animation = show_animation
        self.plot_limits = [0.65, 0.65, 0.8]

        if self.show_animation:
            self.fig = Figure(figsize=(12, 10), dpi=100)
            self.sub1 = self.fig.add_subplot(1, 1, 1, projection="3d")
            self.fig.suptitle("Manipulator Kinematics Visualization", fontsize=16)

        # initialize figure plot
        self.init_plot()

    def init_plot(self):
        """Initializes the plot by calculating the robot's points and calling the plot function."""
        self.robot.calc_robot_points()
        self.plot_3D()

    def update_plot(self, pose=None, angles=None, soln=0, numerical=False):
        """
        Updates the robot's state based on new pose or joint angles and updates the visualization.

        Args:
            pose (EndEffector, optional): Desired end-effector pose for inverse kinematics.
            angles (list, optional): Joint angles for forward kinematics.
            soln (int, optional): The inverse kinematics solution to use (0 or 1).
            numerical (bool, optional): Whether to use numerical inverse kinematics.
        """
        if pose is not None:  # Inverse kinematics case
            if not numerical:
                self.robot.calc_inverse_kinematics(pose, soln=soln)
            else:
                self.robot.calc_numerical_ik(pose, tol=0.02, ilimit=50)
        elif angles is not None:  # Forward kinematics case
            self.robot.calc_forward_kinematics(angles, radians=False)
        else:
            return
        self.plot_3D()

    def move_velocity(self, vel):
        """
        Moves the robot based on a given velocity input.

        Args:
            vel (list): Velocity input for the robot.
        """
        self.robot.calc_velocity_kinematics(vel)
        self.plot_3D()

    def draw_line_3D(self, p1, p2, format_type: str = "k-"):
        """
        Draws a 3D line between two points.

        Args:
            p1 (list): Coordinates of the first point.
            p2 (list): Coordinates of the second point.
            format_type (str, optional): The format of the line. Defaults to "k-".
        """
        self.sub1.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], format_type)

    def draw_ref_line(self, point, axes=None, ref="xyz"):
        """
        Draws reference lines from a given point along specified axes.

        Args:
            point (list): The coordinates of the point to draw from.
            axes (matplotlib.axes, optional): The axes on which to draw the reference lines.
            ref (str, optional): Which reference axes to draw ('xyz', 'xy', or 'xz'). Defaults to 'xyz'.
        """
        line_width = 0.7
        if ref == "xyz":
            axes.plot(
                [point[0], self.plot_limits[0]],
                [point[1], point[1]],
                [point[2], point[2]],
                "b--",
                linewidth=line_width,
            )  # X line
            axes.plot(
                [point[0], point[0]],
                [point[1], self.plot_limits[1]],
                [point[2], point[2]],
                "b--",
                linewidth=line_width,
            )  # Y line
            axes.plot(
                [point[0], point[0]],
                [point[1], point[1]],
                [point[2], 0.0],
                "b--",
                linewidth=line_width,
            )  # Z line
        elif ref == "xy":
            axes.plot(
                [point[0], self.plot_limits[0]],
                [point[1], point[1]],
                "b--",
                linewidth=line_width,
            )  # X line
            axes.plot(
                [point[0], point[0]],
                [point[1], self.plot_limits[1]],
                "b--",
                linewidth=line_width,
            )  # Y line
        elif ref == "xz":
            axes.plot(
                [point[0], self.plot_limits[0]],
                [point[2], point[2]],
                "b--",
                linewidth=line_width,
            )  # X line
            axes.plot(
                [point[0], point[0]], [point[2], 0.0], "b--", linewidth=line_width
            )  # Z line

    def plot_waypoints(self):
        """
        Plots the waypoints in the 3D visualization
        """
        # draw the points
        self.sub1.plot(
            self.waypoint_x, self.waypoint_y, self.waypoint_z, "or", markersize=8
        )

    def update_waypoints(self, waypoints: list):
        """
        Updates the waypoints into a member variable
        """
        for i in range(len(waypoints)):
            self.waypoint_x.append(waypoints[i][0])
            self.waypoint_y.append(waypoints[i][1])
            self.waypoint_z.append(waypoints[i][2])
            # self.waypoint_rotx.append(waypoints[i][3])
            # self.waypoint_roty.append(waypoints[i][4])
            # self.waypoint_rotz.append(waypoints[i][5])

    def plot_3D(self):
        """
        Plots the 3D visualization of the robot, including the robot's links, end-effector, and reference frames.
        """
        self.sub1.cla()
        self.point_x.clear()
        self.point_y.clear()
        self.point_z.clear()

        EE = self.robot.ee

        # draw lines to connect the points
        for i in range(len(self.robot.points) - 1):
            self.draw_line_3D(self.robot.points[i], self.robot.points[i + 1])

        # draw the points
        for i in range(len(self.robot.points)):
            self.point_x.append(self.robot.points[i][0])
            self.point_y.append(self.robot.points[i][1])
            self.point_z.append(self.robot.points[i][2])
        self.sub1.plot(
            self.point_x,
            self.point_y,
            self.point_z,
            marker="o",
            markerfacecolor="m",
            markersize=12,
        )

        # draw the waypoints
        self.plot_waypoints()

        # draw the waypoints
        self.plot_waypoints()

        # draw the EE
        self.sub1.plot(EE.x, EE.y, EE.z, "bo")
        # draw the base reference frame
        self.draw_line_3D(
            self.origin,
            [self.origin[0] + self.axes_length, self.origin[1], self.origin[2]],
            format_type="r-",
        )
        self.draw_line_3D(
            self.origin,
            [self.origin[0], self.origin[1] + self.axes_length, self.origin[2]],
            format_type="g-",
        )
        self.draw_line_3D(
            self.origin,
            [self.origin[0], self.origin[1], self.origin[2] + self.axes_length],
            format_type="b-",
        )
        # draw the EE reference frame
        self.draw_line_3D([EE.x, EE.y, EE.z], self.robot.EE_axes[0], format_type="r-")
        self.draw_line_3D([EE.x, EE.y, EE.z], self.robot.EE_axes[1], format_type="g-")
        self.draw_line_3D([EE.x, EE.y, EE.z], self.robot.EE_axes[2], format_type="b-")
        # draw reference / trace lines
        self.draw_ref_line([EE.x, EE.y, EE.z], self.sub1, ref="xyz")

        # add text at bottom of window
        pose_text = "End-effector Pose:      [ "
        pose_text += f"X: {round(EE.x,4)},  "
        pose_text += f"Y: {round(EE.y,4)},  "
        pose_text += f"Z: {round(EE.z,4)},  "
        pose_text += f"RotX: {round(EE.rotx,4)},  "
        pose_text += f"RotY: {round(EE.roty,4)},  "
        pose_text += f"RotZ: {round(EE.rotz,4)}  "
        pose_text += " ]"

        theta_text = "Joint Positions (deg/m):     ["
        for i in range(self.num_joints):
            theta_text += f" {round(np.rad2deg(self.robot.theta[i]),2)}, "
        theta_text += " ]"

        textstr = pose_text + "\n" + theta_text
        self.sub1.text2D(
            0.2, 0.02, textstr, fontsize=13, transform=self.fig.transFigure
        )

        self.sub1.set_xlim(-self.plot_limits[0], self.plot_limits[0])
        self.sub1.set_ylim(-self.plot_limits[1], self.plot_limits[1])
        self.sub1.set_zlim(0, self.plot_limits[2])
        self.sub1.set_xlabel("x [m]")
        self.sub1.set_ylabel("y [m]")


class TwoDOFRobot:
    """
    Represents a 2-degree-of-freedom (DOF) robot arm with two joints and one end effector.
    Includes methods for calculating forward kinematics (FPK), inverse kinematics (IPK),
    and velocity kinematics (VK).

    Attributes:
        l1 (float): Length of the first arm segment.
        l2 (float): Length of the second arm segment.
        theta (list): Joint angles.
        theta_limits (list): Joint limits for each joint.
        ee (EndEffector): The end effector object.
        points (list): List of points representing the robot's configuration.
        num_dof (int): The number of degrees of freedom (2 for this robot).
    """

    def __init__(self):
        """
        Initializes a 2-DOF robot with default arm segment lengths and joint angles.
        """
        self.l1 = 0.30  # Length of the first arm segment
        self.l2 = 0.25  # Length of the second arm segment

        self.theta = [0.0, 0.0]  # Joint angles (in radians)
        self.theta_limits = [[-PI, PI], [-PI + 0.261, PI - 0.261]]  # Joint limits

        self.ee = EndEffector()  # The end-effector object
        self.num_dof = 2  # Number of degrees of freedom
        self.points = [None] * (self.num_dof + 1)  # List to store robot points

    def calc_forward_kinematics(self, theta: list, radians=False):
        """
        Calculates the forward kinematics for the robot based on the joint angles.

        Args:
            theta (list): Joint angles.
            radians (bool, optional): Whether the angles are in radians or degrees. Defaults to False.
        """

        ########################################

        # insert your code here

        ########################################

        # Update the robot configuration (i.e., the positions of the joints and end effector)
        self.calc_robot_points()

    def calc_inverse_kinematics(self, EE: EndEffector, soln=0):
        """
        Calculates the inverse kinematics (IK) for a given end effector position.

        Args:
            EE (EndEffector): The end effector object containing the target position (x, y).
            soln (int, optional): The solution branch to use. Defaults to 0 (first solution).
        """
        x, y = EE.x, EE.y
        l1, l2 = self.l1, self.l2

        ########################################

        # insert your code here

        ########################################

        # Calculate robot points based on the updated joint angles
        self.calc_robot_points()

    def calc_numerical_ik(self, EE: EndEffector, tol=0.01, ilimit=50):
        """
        Calculates numerical inverse kinematics (IK) based on input end effector coordinates.

        Args:
            EE (EndEffector): The end effector object containing the target position (x, y).
            tol (float, optional): The tolerance for the solution. Defaults to 0.01.
            ilimit (int, optional): The maximum number of iterations. Defaults to 50.
        """

        x, y = EE.x, EE.y

        ########################################

        # insert your code here

        ########################################

        self.calc_robot_points()

    def calc_velocity_kinematics(self, vel: list):
        """
        Calculates the velocity kinematics for the robot based on the given velocity input.

        Args:
            vel (list): The velocity vector for the end effector [vx, vy].
        """

        ########################################

        # insert your code here

        ########################################

        # Update robot points based on the new joint angles
        self.calc_robot_points()

    def calc_robot_points(self):
        """
        Calculates the positions of the robot's joints and the end effector.

        Updates the `points` list, storing the coordinates of the base, shoulder, elbow, and end effector.
        """

        ########################################

        # Replace the placeholder values with your code

        placeholder_value = [0.0, 0.0, 0.0]

        # Base position
        self.points[0] = placeholder_value
        # Shoulder joint
        self.points[1] = placeholder_value
        # Elbow joint
        self.points[2] = placeholder_value

        ########################################

        # Update end effector position
        self.ee.x = self.points[2][0]
        self.ee.y = self.points[2][1]
        self.ee.z = self.points[2][2]
        self.ee.rotz = self.theta[0] + self.theta[1]

        # End effector axes
        self.EE_axes = np.zeros((3, 3))
        self.EE_axes[0] = (
            np.array(
                [
                    cos(self.theta[0] + self.theta[1]),
                    sin(self.theta[0] + self.theta[1]),
                    0,
                ]
            )
            * 0.075
            + self.points[2]
        )
        self.EE_axes[1] = (
            np.array(
                [
                    -sin(self.theta[0] + self.theta[1]),
                    cos(self.theta[0] + self.theta[1]),
                    0,
                ]
            )
            * 0.075
            + self.points[2]
        )
        self.EE_axes[2] = np.array([0, 0, 1]) * 0.075 + self.points[2]


class ScaraRobot:
    """
    A class representing a SCARA (Selective Compliance Assembly Robot Arm) robot.
    This class handles the kinematics (forward, inverse, and velocity kinematics)
    and robot configuration, including joint limits and end-effector calculations.
    """

    def __init__(self):
        """
        Initializes the SCARA robot with its geometry, joint variables, and limits.
        Sets up the transformation matrices and robot points.
        """
        # Geometry of the robot (link lengths in meters)
        self.l1 = 0.35  # Base to 1st joint
        self.l2 = 0.18  # 1st joint to 2nd joint
        self.l3 = 0.15  # 2nd joint to 3rd joint
        self.l4 = 0.30  # 3rd joint to 4th joint (tool or end-effector)
        self.l5 = 0.12  # Tool offset

        # Joint variables (angles in radians)
        self.theta = [0.0, 0.0, 0.0]

        # Joint angle limits (min, max) for each joint
        self.theta_limits = [
            [-np.pi, np.pi],
            [-np.pi + 0.261, np.pi - 0.261],
            [0, self.l1 + self.l3 - self.l5],
        ]

        # End-effector (EE) object to store EE position and orientation
        self.ee = EndEffector()

        # Number of degrees of freedom and number of points to store robot configuration
        self.num_dof = 3
        self.num_points = 7
        self.points = [None] * self.num_points

        # Transformation matrices (DH parameters and resulting transformation)
        self.DH = np.zeros((5, 4))  # Denavit-Hartenberg parameters (theta, d, a, alpha)
        self.T = np.zeros((self.num_dof, 4, 4))  # Transformation matrices

        ########################################

        # insert your additional code here

        ########################################

    def calc_forward_kinematics(self, theta: list, radians=False):
        """
        Calculate Forward Kinematics (FK) based on the given joint angles.

        Args:
            theta (list): Joint angles (in radians if radians=True, otherwise in degrees).
            radians (bool): Whether the input angles are in radians (default is False).
        """
        ########################################

        # insert your code here

        ########################################

        # Calculate robot points (e.g., end-effector position)
        self.calc_robot_points()

    def calc_inverse_kinematics(self, EE: EndEffector, soln=0):
        """
        Calculate Inverse Kinematics (IK) based on the input end-effector coordinates.

        Args:
            EE (EndEffector): End-effector object containing desired position (x, y, z).
            soln (int): Solution index (0 or 1), for multiple possible IK solutions.
        """
        x, y, z = EE.x, EE.y, EE.z
        l1, l2, l3, l4, l5 = self.l1, self.l2, self.l3, self.l4, self.l5

        ########################################

        # insert your code here

        ########################################

        # Recalculate Forward Kinematics to update the robot's configuration
        self.calc_forward_kinematics(self.theta, radians=True)

    def calc_velocity_kinematics(self, vel: list):
        """
        Calculate velocity kinematics and update joint velocities.

        Args:
            vel (array): Linear velocities (3D) of the end-effector.
        """
        ########################################

        # insert your code here

        ########################################

        # Recalculate robot points based on updated joint angles
        self.calc_robot_points()

    def calc_robot_points(self):
        """
        Calculate the main robot points (links and end-effector position) using the current joint angles.
        Updates the robot's points array and end-effector position.
        """

        # Calculate transformation matrices for each joint and end-effector
        self.points[0] = np.array([0, 0, 0, 1])
        self.points[1] = np.array([0, 0, self.l1, 1])
        self.points[2] = self.T[0] @ self.points[0]
        self.points[3] = self.points[2] + np.array([0, 0, self.l3, 1])
        self.points[4] = self.T[0] @ self.T[1] @ self.points[0] + np.array(
            [0, 0, self.l5, 1]
        )
        self.points[5] = self.T[0] @ self.T[1] @ self.points[0]
        self.points[6] = self.T[0] @ self.T[1] @ self.T[2] @ self.points[0]

        self.EE_axes = (
            self.T[0] @ self.T[1] @ self.T[2] @ np.array([0.075, 0.075, 0.075, 1])
        )
        self.T_ee = self.T[0] @ self.T[1] @ self.T[2]

        # End-effector (EE) position and axes
        self.ee.x = self.points[-1][0]
        self.ee.y = self.points[-1][1]
        self.ee.z = self.points[-1][2]
        rpy = rotm_to_euler(self.T_ee[:3, :3])
        self.ee.rotx, self.ee.roty, self.ee.rotz = rpy

        # EE coordinate axes
        self.EE_axes = np.zeros((3, 3))
        self.EE_axes[0] = self.T_ee[:3, 0] * 0.075 + self.points[-1][0:3]
        self.EE_axes[1] = self.T_ee[:3, 1] * 0.075 + self.points[-1][0:3]
        self.EE_axes[2] = self.T_ee[:3, 2] * 0.075 + self.points[-1][0:3]


class FiveDOFRobot:
    """
    A class to represent a 5-DOF robotic arm with kinematics calculations, including
    forward kinematics, inverse kinematics, velocity kinematics, and Jacobian computation.

    Attributes:
        l1, l2, l3, l4, l5: Link lengths of the robotic arm.
        theta: List of joint angles in radians.
        theta_limits: Joint limits for each joint.
        ee: End-effector object for storing the position and orientation of the end-effector.
        num_dof: Number of degrees of freedom (5 in this case).
        points: List storing the positions of the robot joints.
        DH: Denavit-Hartenberg parameters for each joint.
        T: Transformation matrices for each joint.
    """

    def __init__(self):
        """Initialize the robot parameters and joint limits."""
        # Link lengths
        self.l1, self.l2, self.l3, self.l4, self.l5 = 0.30, 0.15, 0.18, 0.15, 0.12

        # Joint angles (initialized to zero)
        self.theta = [0, 0, 0, 0, 0]

        # Joint limits (in radians)
        self.theta_limits = [
            [-np.pi, np.pi],
            [-np.pi / 3, np.pi],
            [-np.pi + np.pi / 12, np.pi - np.pi / 4],
            [-np.pi + np.pi / 12, np.pi - np.pi / 12],
            [-np.pi, np.pi],
        ]

        # End-effector object
        self.ee = EndEffector()

        # Robot's points
        self.num_dof = 5
        self.points = [None] * (self.num_dof + 1)

        # Denavit-Hartenberg parameters and transformation matrices
        self.DH = np.zeros((5, 4))
        self.T = np.zeros((self.num_dof, 4, 4))

        ########################################

        # insert your additional code here

        ########################################

    def calc_forward_kinematics(self, theta: list, radians=False):
        """
        Calculate forward kinematics based on the provided joint angles.

        Args:
            theta: List of joint angles (in degrees or radians).
            radians: Boolean flag to indicate if input angles are in radians.
        """
        if not radians:
            theta = np.radians(theta)

        self.theta = theta

        for joint, limits in enumerate(self.theta_limits):
            if self.theta[joint] < limits[0]:
                self.theta[joint] = limits[0]
            elif self.theta[joint] > limits[1]:
                self.theta[joint] = limits[1]

        self.DH = [
            [self.theta[0], self.l1, 0, np.pi / 2],
            [(np.pi / 2) + self.theta[1], 0, self.l2, 0],
            [-self.theta[2], 0, self.l3, 0],
            [self.theta[3], 0, self.l4 + self.l5, self.theta[4]],
            [(-np.pi / 2), 0, 0, (-np.pi / 2)],
        ]

        for index, dh_item in enumerate(self.DH):
            self.T[index] = [
                [
                    cos(dh_item[0]),
                    -sin(dh_item[0]) * cos(dh_item[3]),
                    sin(dh_item[0]) * sin(dh_item[3]),
                    dh_item[2] * cos(dh_item[0]),
                ],
                [
                    sin(dh_item[0]),
                    cos(dh_item[0]) * cos(dh_item[3]),
                    -cos(dh_item[0]) * sin(dh_item[3]),
                    dh_item[2] * sin(dh_item[0]),
                ],
                [0, sin(dh_item[3]), cos(dh_item[3]), dh_item[1]],
                [0, 0, 0, 1],
            ]

        # Calculate robot points (positions of joints)
        self.calc_robot_points()

    def calc_inverse_kinematics(self, EE: EndEffector, soln=0):
        """
        Calculate inverse kinematics to determine the joint angles based on end-effector position.

        Args:
            EE: EndEffector object containing desired position and orientation.
            soln: Optional parameter for multiple solutions.
        """

        x, y, z = EE.x, EE.y, EE.z

        R_05 = euler_to_rotm((EE.rotx, EE.roty, EE.rotz))
        R_05_K = R_05 @ K

        p_wrist = [x, y, z] - ((self.l4 + self.l5) * (R_05_K))

        wx = p_wrist[0]
        wy = p_wrist[1]
        wz = p_wrist[2]

        r = sqrt((wx**2 + wy**2) + ((wz - self.l1) ** 2))

        # Theta 1 standard solve
        j1 = wraptopi(atan2(y, x) + np.pi)  # seems to give desired - 180

        # Theta 2 standard solve
        alpha = acos((r**2 + self.l2**2 - self.l3**2) / (2 * r * self.l2))
        phi = acos((wz - self.l1) / (r))
        j2 = alpha + phi

        # Theta 3 standard solve
        j3 = acos((r**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3))

        # Set of 4 potential solutions
        #! @EarlJr53 interested in adding more solutions perhaps?
        solns = np.array(
            [
                # Standard configuration
                [j1, j2, j3, 0, 0],
                # Flipped elbow
                [j1, -alpha + phi, -j3, 0, 0],
                # Mirrored base
                [wraptopi(j1 + PI), -alpha - phi, -j3, 0, 0],
                # Mirrored base, flipped elbow
                [wraptopi(j1 + PI), alpha - phi, j3, 0, 0],
            ]
        )

        # Keep track of how many valid solutions have been "skipped"
        valid_solns_count = 0

        for angs in solns:
            # Temporary DH matrix
            mini_DH = [
                [angs[0], self.l1, 0, np.pi / 2],
                [(np.pi / 2) + angs[1], 0, self.l2, 0],
                [-angs[2], 0, self.l3, 0],
            ]

            # Translation matrix from 0 to 3
            t_03 = np.eye(4)
            for dh_item in mini_DH:
                t_temp = dh_to_matrix(dh_item)
                t_03 = t_03 @ t_temp

            # Rotation matrix from 0 to 3
            R_03 = t_03[:3, :3]

            # Rotation matrix from 3 to 5
            R_35 = np.transpose(R_03) @ R_05

            # Solve for Theta 4 and Theta 5
            angs[3] = atan2(R_35[1, 2], R_35[0, 2])
            angs[4] = wraptopi(atan2(R_35[2, 0], R_35[2, 1]) + PI)

            # Check if the current configuration is valid
            if check_joint_limits(angs, self.theta_limits):
                # Check if we've reached either the requested `soln` or the last valid solution
                if soln is valid_solns_count or valid_solns_count is len(solns) - 1:
                    self.theta = angs
                    break
                # "Skip" valid solutions until the requested `soln`
                else:
                    last_valid = angs  # Hang onto the most recent valid solution
                    valid_solns_count += 1
            # If we've made it to the end and don't have another valid solution, use the most recent valid one
            elif valid_solns_count is len(solns) - 1:
                self.theta = last_valid

        # Include this to automatically activate FPK
        self.calc_forward_kinematics(self.theta, True)

        ########################################

    def calc_numerical_ik(self, EE: EndEffector, tol=0.01, ilimit=50):
        """Calculate numerical inverse kinematics based on input coordinates."""

        ########################################

        # insert your code here

        ########################################
        self.calc_forward_kinematics(self.theta, radians=True)

    def calc_velocity_kinematics(self, vel: list):
        """
        Calculate the joint velocities required to achieve the given end-effector velocity.

        Args:
            vel: Desired end-effector velocity (3x1 vector).
        """

        Jv = np.zeros((self.num_dof, 3))
        H0_n = np.eye(4)
        for i in range(len(self.T)):
            H0_n = H0_n @ self.T[i]
        r0_n = H0_n[:3, 3]

        # Compute the jacobian
        for i in range(self.num_dof):
            H0_i = np.eye(4)
            for j in range(i):
                H0_i = H0_i @ self.T[j]

            R0_i = H0_i[:3, :3]
            r0_i = H0_i[:3, 3]
            ri_n = r0_n - r0_i

            zi = R0_i @ np.array([[0], [0], [1]])
            zi_T = np.transpose(zi)

            Jvi = np.cross(zi_T, ri_n)
            Jv[i] = Jvi[0]

        Jv = np.transpose(Jv)
        print(Jv)

        # (pseudo) Invert jacobian
        Jv_inv = np.linalg.pinv(Jv)

        # Compute joint velocities

        dth_k = Jv_inv @ vel

        print("dth_k")
        print(dth_k)
        for i in range(len(dth_k)):
            if dth_k[i] > 3:
                dth_k = dth_k * 3 / dth_k[i]
            elif dth_k[i] < -3:
                dth_k = dth_k * -3 / dth_k[i]
        print(dth_k)

        # Compute the next joint config
        dt = 0.03  # from main_arm.py 273
        self.theta = np.add(self.theta, np.multiply(dt, dth_k))

        # Recompute robot points based on updated joint angles
        self.calc_forward_kinematics(self.theta, radians=True)

    def calc_robot_points(self):
        """Calculates the main arm points using the current joint angles"""

        # Initialize points[0] to the base (origin)
        self.points[0] = np.array([0, 0, 0, 1])

        # Precompute cumulative transformations to avoid redundant calculations
        T_cumulative = [np.eye(4)]
        for i in range(len(self.T)):
            T_cumulative.append(T_cumulative[-1] @ self.T[i])

        # Calculate the robot points by applying the cumulative transformations
        for i in range(1, self.num_dof + 1):
            self.points[i] = T_cumulative[i] @ self.points[0]

        # Calculate EE position and rotation
        self.EE_axes = T_cumulative[-1] @ np.array(
            [0.075, 0.075, 0.075, 1]
        )  # End-effector axes
        self.T_ee = T_cumulative[-1]  # Final transformation matrix for EE

        # Set the end effector (EE) position
        self.ee.x, self.ee.y, self.ee.z = self.points[-1][:3]

        # Extract and assign the RPY (roll, pitch, yaw) from the rotation matrix
        rpy = rotm_to_euler(self.T_ee[:3, :3])
        self.ee.rotx, self.ee.roty, self.ee.rotz = rpy[0], rpy[1], rpy[2]

        # Calculate the EE axes in space (in the base frame)
        self.EE = [self.ee.x, self.ee.y, self.ee.z]
        self.EE_axes = np.array(
            [self.T_ee[:3, i] * 0.075 + self.points[-1][:3] for i in range(3)]
        )
