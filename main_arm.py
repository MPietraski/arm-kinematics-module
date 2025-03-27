
import sys
sys.path.append('../')
import argparse
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from arm_models import Robot
from helper_fcns.utils import EndEffector
import time
from pynput import keyboard
import yaml


class Visualizer:
    """
    A class for visualizing and controlling a robot manipulator, including forward and inverse kinematics, 
    and velocity control through a Tkinter GUI.
    """

    def __init__(self, root, args):
        """
        Initializes the Visualizer class, setting up the Tkinter GUI and creating the robot instance.

        Args:
            root (tk.Tk): The Tkinter root window.
            args (argparse.Namespace): The command-line arguments.
        """
        self.root = root
        title = f"Robot Manipulator Visualization for a {args.robot_type} robot"
        self.root.title(title)

        self.robot_type = args.robot_type
        self.robot = Robot(type=self.robot_type)

        # Variables for velocity kinematics
        self.vk_status = False
        self.vk_status_font = ('black')

        # Keyboard listener for controlling velocity
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.v = [0, 0, 0]

        # Create the control frame for the GUI
        self.control_frame = ttk.Frame(root)
        self.control_frame.grid(row=0, column=0, padx=15, pady=15)

        # Set up the kinematics panel
        self.set_kinematics_panel()

        # Create the plot frame
        self.plot_frame = ttk.Frame(root)
        self.plot_frame.grid(row=0, column=1, padx=10, pady=10)

        # Embed the robot's figure into the Tkinter canvas
        self.canvas = FigureCanvasTkAgg(self.robot.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0)

        
    def set_kinematics_panel(self):
        """
        Sets up the control panel for forward kinematics, inverse kinematics, and velocity kinematics.

        This method creates entry fields, buttons, and sliders for controlling the robot's joints,
        as well as buttons for solving inverse kinematics and activating/deactivating velocity kinematics.
        """
        # ------------------------------------------------------------------------------------------------
        # Forward position kinematics
        # ------------------------------------------------------------------------------------------------
        self.joint_values = []
        row_number = 0

        # Add title for the forward kinematics entry field
        self.fk_entry_title = ttk.Label(self.control_frame, text="Forward Kinematics:", font=("Arial", 13, "bold"))
        self.fk_entry_title.grid(column=0, row=row_number, columnspan=2, pady=(0, 10))
        row_number += 1

        # Create joint entry fields and labels
        self.joint_button = []
        for i in range(self.robot.num_joints):
            joint_label = ttk.Label(self.control_frame, text=f"theta {i+1} (deg or m):")
            joint_label.grid(column=0, row=row_number, sticky=tk.W)
            joint_value = ttk.Entry(self.control_frame)
            joint_value.insert(0, "0")
            joint_value.grid(column=1, row=row_number)
            self.joint_button.append(joint_value)
            row_number += 1

        # Create the Move button
        self.fk_move_button = ttk.Button(self.control_frame, text="Move", command=self.joints_from_button)
        self.fk_move_button.grid(column=0, row=row_number, columnspan=2, pady=5)
        row_number += 1

        # Create the joint slider field and labels
        self.joint_scales = []
        for i in range(self.robot.num_joints):
            joint_label = ttk.Label(self.control_frame, text=f"theta {i+1} (deg or m):")
            joint_label.grid(column=0, row=row_number, sticky=tk.W)

            joint_value = tk.DoubleVar()
            slider = ttk.Scale(self.control_frame, from_=-180, to=180, variable=joint_value, command=self.joints_from_sliders)
            slider.grid(column=1, row=row_number)
            row_number += 1
            self.joint_scales.append(joint_value)

        # Create the Reset button
        self.fk_reset_button = ttk.Button(self.control_frame, text="Reset", command=self.reset_joints)
        self.fk_reset_button.grid(column=0, row=row_number, columnspan=2, pady=5)
        row_number += 3

        # ------------------------------------------------------------------------------------------------
        # Inverse position kinematics
        # ------------------------------------------------------------------------------------------------
        self.ik_entry_title = ttk.Label(self.control_frame, text="Inverse Kinematics:", font=("Arial", 13, "bold"))
        self.ik_entry_title.grid(column=0, row=row_number, columnspan=2, pady=(0, 10))
        row_number += 1

        # Create end-effector pose field and labels
        self.pose_button = []
        pose_labels = ['X(m)', 'Y(m)', 'Z(m)', 'RotX(rad)', 'RotY(rad)', 'RotZ(rad)']
        for i in range(len(pose_labels)):
            position_label = ttk.Label(self.control_frame, text=pose_labels[i] + ":")
            position_label.grid(column=0, row=row_number, sticky=tk.W)
            position_value = ttk.Entry(self.control_frame)
            position_value.insert(0, "0")
            position_value.grid(column=1, row=row_number)
            row_number += 1
            self.pose_button.append(position_value)

        # Create buttons for inverse kinematics solutions
        self.ik1_move_button = ttk.Button(self.control_frame, text="Solve 1", command=self.solve_IK1)
        self.ik1_move_button.grid(column=0, row=row_number, columnspan=1, pady=2)

        self.ik2_move_button = ttk.Button(self.control_frame, text="Solve 2", command=self.solve_IK2)
        self.ik2_move_button.grid(column=1, row=row_number, columnspan=1, pady=2)

        self.ik3_move_button = ttk.Button(self.control_frame, text="Num Solve", command=self.numerical_solve)
        self.ik3_move_button.grid(column=2, row=row_number, columnspan=1, pady=2)
        row_number += 3

        # ------------------------------------------------------------------------------------------------
        # Velocity kinematics
        # ------------------------------------------------------------------------------------------------
        self.vk_entry_title = ttk.Label(self.control_frame, text="Velocity Kinematics:", font=("Arial", 13, "bold"))
        self.vk_entry_title.grid(column=0, row=row_number, columnspan=2, pady=(0, 10))
        row_number += 1

        self.vk_activate_button = ttk.Button(self.control_frame, text="Activate VK", command=self.activate_VK)
        self.vk_activate_button.grid(column=0, row=row_number, columnspan=1, pady=2)

        self.vk_deactivate_button = ttk.Button(self.control_frame, text="Deactivate VK", command=self.deactivate_VK)
        self.vk_deactivate_button.grid(column=1, row=row_number, columnspan=1, pady=2)
        row_number += 3

        # ------------------------------------------------------------------------------------------------
        # Trajectory generation
        # ------------------------------------------------------------------------------------------------
        self.tg_entry_title = ttk.Label(self.control_frame, text="Trajectory Generation:", font=("Arial", 13, "bold"))
        self.tg_entry_title.grid(column=0, row=row_number, columnspan=2, pady=(0, 10))
        row_number += 1

        self.tg_generate_button = ttk.Button(self.control_frame, text="Generate Trajectory", command=self.generate_traj)
        self.tg_generate_button.grid(column=0, row=row_number, columnspan=1, pady=2)

        self.tg_folow_button = ttk.Button(self.control_frame, text="Follow Trajectory", command=self.follow_traj)
        self.tg_folow_button.grid(column=1, row=row_number, columnspan=1, pady=2)
        row_number += 1

        # Start the keyboard listener
        self.listener.start()


    def joints_from_sliders(self, val):
        """
        Updates the forward kinematics based on the joint angles set by the sliders.

        Args:
            val (str): The value of the slider, not used in this method at this moment.
        """
        theta = [float(th.get()) for th in self.joint_scales]
        self.update_FK(theta)


    def joints_from_button(self):
        """
        Updates the forward kinematics based on the joint angles entered in the input fields.
        """
        theta = [float(th.get()) for th in self.joint_button]
        self.update_FK(theta)


    def reset_joints(self):
        """
        Resets all joint angles to 0 and updates the forward kinematics.
        """
        theta = [0.0] * self.robot.num_joints
        self.update_FK(theta)


    def solve_IK1(self):
        """
        Solves the inverse kinematics for a given end-effector pose using the first solution.
        """
        EE = EndEffector()
        EE.x = float(self.pose_button[0].get())
        EE.y = float(self.pose_button[1].get())
        EE.z = float(self.pose_button[2].get())
        EE.rotx = float(self.pose_button[3].get())
        EE.roty = float(self.pose_button[4].get())
        EE.rotz = float(self.pose_button[5].get())

        self.update_IK(pose=EE, soln=0)


    def solve_IK2(self):
        """
        Solves the inverse kinematics for a given end-effector pose using the second solution.
        """
        EE = EndEffector()
        EE.x = float(self.pose_button[0].get())
        EE.y = float(self.pose_button[1].get())
        EE.z = float(self.pose_button[2].get())
        EE.rotx = float(self.pose_button[3].get())
        EE.roty = float(self.pose_button[4].get())
        EE.rotz = float(self.pose_button[5].get())

        self.update_IK(pose=EE, soln=1)


    def numerical_solve(self):
        """
        Solves the inverse kinematics for a given end-effector pose using a numerical method.
        """
        EE = EndEffector()
        EE.x = float(self.pose_button[0].get())
        EE.y = float(self.pose_button[1].get())
        EE.z = float(self.pose_button[2].get())
        EE.rotx = float(self.pose_button[3].get())
        EE.roty = float(self.pose_button[4].get())
        EE.rotz = float(self.pose_button[5].get())

        self.update_IK(pose=EE, soln=1, numerical=True)


    def update_FK(self, theta: list):
        """
        Updates the forward kinematics plot based on the given joint angles.

        Args:
            theta (list): List of joint angles.
        """
        try:
            self.robot.update_plot(angles=theta)
            self.canvas.draw()
        except ValueError:
            tk.messagebox.showerror("Input Error", "Please enter valid numbers")


    def update_IK(self, pose: EndEffector, soln=0, numerical=False):
        """
        Updates the inverse kinematics plot based on the given end-effector pose.

        Args:
            pose (EndEffector): The desired end-effector pose.
            soln (int, optional): The solution index to use. Defaults to 0.
            numerical (bool, optional): Whether to use a numerical solver. Defaults to False.
        """
        if numerical:
            self.robot.update_plot(pose=pose, soln=soln, numerical=True)
        else:
            self.robot.update_plot(pose=pose, soln=soln)

        self.canvas.draw()


    def activate_VK(self):
        """
        Activates velocity kinematics and starts continuously updating the robot's movement.
        """
        self.vk_status = True
        while self.vk_status:
            self.robot.move_velocity(self.v)
            self.canvas.draw()
            self.canvas.flush_events()
            time.sleep(0.05)


    def deactivate_VK(self):
        """
        Deactivates velocity kinematics, stopping the robot's movement.
        """
        self.vk_status = False


    def generate_traj(self):
        """
        Reads waypoint data from yaml file and plots points in 3D visualization
        """
        print('Trajectory Generation Mode Activated! \n Generating trajectory...')

        # get pid_gains from yaml file
        with open('waypoints.yml', 'r') as file:
            waypoints = yaml.safe_load(file)

        self.waypoint_idx = 0
        self.robot.update_waypoints(waypoints['points'])
        self.robot.plot_3D()
        self.canvas.draw()

    
    def follow_traj(self):
        """
        TBA
        """
        pass



    def check_vk_status(self):
        """
        Checks and returns the status of the velocity kinematics.

        Returns:
            str: The status of velocity kinematics ("Activated!" or "Deactivated!").
        """
        return 'Deactivated!' if not self.vk_status else 'Activated!'


    def on_press(self, key):
        """
        Handles key press events to control the velocity of the robot.

        Args:
            key (pynput.keyboard.Key): The key that was pressed.
        """
        if self.vk_status:
            if key == keyboard.Key.up:
                self.v[1] = 1
            elif key == keyboard.Key.down:
                self.v[1] = -1
            elif key == keyboard.Key.left:
                self.v[0] = -1
            elif key == keyboard.Key.right:
                self.v[0] = 1
            elif hasattr(key, 'char'):
                if key.char == 'w':
                    self.v[2] = 1
                elif key.char == 's':
                    self.v[2] = -1


    def on_release(self, key):
        """
        Handles key release events to stop the robot's movement.

        Args:
            key (pynput.keyboard.Key): The key that was released.
        """
        if key == keyboard.Key.up:
            self.v[1] = 0
        elif key == keyboard.Key.down:
            self.v[1] = 0
        elif key == keyboard.Key.left:
            self.v[0] = 0
        elif key == keyboard.Key.right:
            self.v[0] = 0
        elif hasattr(key, 'char'):
            if key.char == 'w':
                self.v[2] = 0
            elif key.char == 's':
                self.v[2] = 0



def get_robot_type(robot_type: str) -> str:
    """
    Maps the robot type argument to a human-readable string.
    
    Args:
        robot_type (str): The robot type provided as input, e.g., '2-dof', 'scara', '5-dof'.
        
    Returns:
        str: The corresponding robot type in a more readable format.
        
    Raises:
        ValueError: If an unsupported robot type is provided.
    """
    if robot_type == '2-dof':
        return 'Two-DOF'
    elif robot_type == 'scara':
        return 'Scara'
    elif robot_type == '5-dof':
        return 'Five-DOF'
    else:
        raise ValueError(f"[ERROR] Unsupported robot type '{robot_type}'. Please provide one of the available types ('2-dof', 'scara', '5-dof').")



def main():
    """
    Main function to initialize the Robot Manipulator Visualization application.
    
    Parses the robot type argument from the command line and initializes the
    visualization for the corresponding robot type.
    """
    # Argument parsing
    parser = argparse.ArgumentParser(description="Robot Manipulator Visualization for Kinematics Analysis")
    parser.add_argument('--robot_type', 
                        help="Insert robot type, e.g., '2-dof', 'scara', '5-dof'. Default is '2-dof'.", 
                        default='2-dof')
    args = parser.parse_args()

    try:
        # Map the robot type to a readable format
        robot_type = get_robot_type(args.robot_type)
    except ValueError as e:
        print(str(e))
        return
    
    # Initialization message
    print(f"\nInitialized the Robot Manipulator Visualization for [ {robot_type} robot ] for Kinematics Analysis\n")

    # Create the Tkinter root window and run the visualizer application
    root = tk.Tk()
    app = Visualizer(root, args)
    root.mainloop()




if __name__ == "__main__":
    main()