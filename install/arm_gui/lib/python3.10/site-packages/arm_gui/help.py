import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class gui_run(Node):
    def __init__(self):
        super().__init__('gui_run')

        # ROS2 Publisher & Subscriber
        self.publisher = self.create_publisher(Float32MultiArray, 'joint_angles', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'joint_angles', self.listener_callback, 10)

        # Tkinter Window Setup
        self.root = tk.Tk()
        self.root.title("Robot Arm Visualizer")
        self.root.geometry("900x900")
        self.root.resizable(False,False)

        # Matplotlib Figures for Arm Visualization
        self.fig, (self.ax_top, self.ax_front) = plt.subplots(1, 2, figsize=(10, 5))

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Initialize Joint Values
        self.joint_values = [0, 0, 0, 0]  # Base, Shoulder, Elbow, Wrist
        labels = ["Base (Top View)", "Shoulder", "Elbow", "Wrist"]
        self.sliders = []

        for i in range(4):
            frame = tk.Frame(self.root)
            frame.pack(fill=tk.X)

            label = tk.Label(frame, text=labels[i])
            label.pack(side=tk.LEFT)

            slider = ttk.Scale(frame, from_=-180, to=180, orient="horizontal", length=400,
                               command=lambda val, idx=i: self.update_angle(idx, val))
            slider.pack(side=tk.RIGHT)
            slider.set(0)
            self.sliders.append(slider)

        # Start the GUI Update Loop
        self.update_visualization()
        self.root.after(100, self.update_loop)
        self.root.mainloop()

    def update_angle(self, index, value):
        """ Updates joint angle when a slider is moved. """
        self.joint_values[index] = float(value)
        self.publish_joint_angles()

    def publish_joint_angles(self):
        """ Publishes joint angles to the /joint_angles topic. """
        msg = Float32MultiArray()
        msg.data = self.joint_values
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint angles: {self.joint_values}")

    def listener_callback(self, msg):
        """ Updates GUI sliders when new joint angles are received. """
        self.joint_values = list(msg.data)
        for i, slider in enumerate(self.sliders):
            slider.set(self.joint_values[i])
        self.update_visualization()

    def update_visualization(self):
        """ Updates the arm visualization based on joint angles. """
        self.ax_top.clear()
        self.ax_front.clear()

        self.ax_top.set_aspect('equal')
        self.ax_front.set_aspect('equal')



        self.ax_top.set_xlim(-1.5, 1.5)
        self.ax_top.set_ylim(-1.5, 1.5)
        self.ax_top.set_title("Top View (XY) - Base Rotation")

        self.ax_front.set_xlim(-3, 3)
        self.ax_front.set_ylim(-3, 3)
        self.ax_front.set_title("Front View (XZ) - Arm Motion")

        # Arm Kinematics (Top View - Base Rotation)
        base_x, base_y = 0, 0
        link1_x = np.cos(np.radians(self.joint_values[0])) * 1.0
        link1_y = np.sin(np.radians(self.joint_values[0])) * 1.0

        self.ax_top.plot([base_x, link1_x], [base_y, link1_y], marker="o", color="red", linewidth=3)

        # Arm Kinematics (Front View - Shoulder, Elbow, Wrist)
        shoulder_x, shoulder_y = 0, 1
        elbow_x = shoulder_x + np.cos(np.radians(self.joint_values[1])) * 1.0
        elbow_y = shoulder_y + np.sin(np.radians(self.joint_values[1])) * 1.0

        wrist_x = elbow_x + np.cos(np.radians(self.joint_values[2])) * 0.8
        wrist_y = elbow_y + np.sin(np.radians(self.joint_values[2])) * 0.8

        end_x = wrist_x + np.cos(np.radians(self.joint_values[3])) * 0.6
        end_y = wrist_y + np.sin(np.radians(self.joint_values[3])) * 0.6

        self.ax_front.plot([shoulder_x, elbow_x, wrist_x, end_x],
                           [shoulder_y, elbow_y, wrist_y, end_y],
                           marker="o", color="blue", linewidth=3)

        self.canvas.draw()

    def update_loop(self):
        """ Loops GUI updates. """
        self.update_visualization()
        self.root.after(100, self.update_loop)

def main(args=None):
    rclpy.init(args=args)
    gui_node = gui_run()
    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()