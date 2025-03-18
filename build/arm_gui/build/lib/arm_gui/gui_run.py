import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class gui_run(Node):
    def __init__(self):
        super().__init__('gui_run')
        self.pub_ = self.create_publisher(Float32MultiArray, 'joint_angles', 10)
        self.sub_ = self.create_subscription(Float32MultiArray, 'joint_angles', self.listener_callback, 10)

        # Tkinter_window_setup
        self.root = tk.Tk()
        self.root.title("ROBOT ARM CONTROL")
        self.root.geometry("1000x1000")
        self.root.configure(bg="#EAEDED")
        self.root.resizable(False, False)



        self.fig, (self.ax_top, self.ax_front) = plt.subplots(1, 2, figsize=(10, 5))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        heading_label = tk.Label(self.root, text="POSITION", font=("Helvetica", 18, "bold"))
        heading_label.pack(pady=(10, 10))



        self.joint_values = [0, 0, 0, 0]  
        labels = ["Base", "Shoulder", "Elbow", "Wrist"]
        self.sliders = []
        self.entries = []


        self.controls_frame = tk.Frame(self.root, bg="#EAEDED")
        self.controls_frame.pack(pady=10)




        for i in range(4):
            frame = tk.Frame(self.controls_frame,bg="#EAEDED")
            frame.pack(anchor='center', pady=5)

            tk.Label(frame, text=labels[i],width=14,anchor="center",font=("Helvetica",12)).pack(side=tk.LEFT, padx=5)

            slider = tk.Scale(frame, from_=0, to=180, orient="horizontal", length=200,
                               command=lambda val, idx=i: self.update_angle(idx, val))
            slider.pack(side=tk.LEFT, padx=5)
            slider.set(0)
            self.sliders.append(slider)


            tk.Button(frame, text="◀",width=2, command=lambda idx=i: self.move_slider(idx, -1)).pack(side=tk.LEFT, padx=2)


            tk.Button(frame, text="▶",width=2, command=lambda idx=i: self.move_slider(idx, 1)).pack(side=tk.LEFT, padx=2)

            entry = tk.Entry(frame, width=5,justify="center")
            entry.pack(side=tk.LEFT, padx=5)
            entry.insert(0, "0")
            entry.bind("<Return>", lambda event, idx=i: self.set_angle_from_entry(idx))

            self.entries.append(entry)


        self.update_visualization()
        self.root.after(100, self.update_loop)
        self.root.mainloop()

    def update_angle(self, index, value):
        """Update angle from slider movement."""
        self.joint_values[index] = int(float(value))
        self.entries[index].delete(0, tk.END)
        self.entries[index].insert(0, str(self.joint_values[index]))
        self.publish_joint_angles()
        self.update_visualization()

    def set_angle_from_entry(self, index):
        """Set slider from entry field when Enter is pressed."""
        try:
            value = int(self.entries[index].get())
            if 0 <= value <= 180:
                self.sliders[index].set(value)
                self.update_angle(index, value)
            else:
                self.entries[index].delete(0, tk.END)
                self.entries[index].insert(0, str(self.joint_values[index]))
        except ValueError:
            self.entries[index].delete(0, tk.END)
            self.entries[index].insert(0, str(self.joint_values[index]))

    def move_slider(self, index, direction, step=5):

        new_value = self.sliders[index].get() + direction * step
        new_value = max(0, min(180, new_value))
        self.sliders[index].set(new_value)
        self.update_angle(index, new_value)

    def publish_joint_angles(self):

        msg = Float32MultiArray()
        msg.data = self.joint_values
        self.pub_.publish(msg)
        self.get_logger().info(f"Published: {self.joint_values}")

    def listener_callback(self, msg):

        self.joint_values = list(map(int, msg.data))
        for i in range(4):
            self.sliders[i].set(self.joint_values[i])
            self.entries[i].delete(0, tk.END)
            self.entries[i].insert(0, str(self.joint_values[i]))
        self.update_visualization()

    def update_visualization(self):

        self.ax_top.clear()
        self.ax_front.clear()
        #adddd
        self.ax_top.grid(True, linestyle='--', alpha=0.5)
        self.ax_front.grid(True, linestyle='--', alpha=0.5)
        self.ax_top.set_facecolor("#f0f0f0")     # Light grey
        self.ax_front.set_facecolor("#f0f0f0")



        self.ax_top.set_aspect('equal')
        self.ax_front.set_aspect('equal')
        

        self.ax_top.set_xlim(-1.5, 1.5)
        self.ax_top.set_ylim(-1.5, 1.5)
        self.ax_top.set_title("Top View (XY) - Base Rotation",fontsize=14,fontweight='bold')
        self.ax_top.tick_params(axis='both', which='major', labelsize=10)


        self.ax_front.set_xlim(-3, 3)
        self.ax_front.set_ylim(-3, 3)
        self.ax_front.set_title("Front View (XZ) - Arm Motion",fontsize=14,fontweight='bold')
        self.ax_front.tick_params(axis='both', which='major', labelsize=10)

        base_x, base_y = 0, 0
        link1_x = np.cos(np.radians(self.joint_values[0])) * 1.0
        link1_y = np.sin(np.radians(self.joint_values[0])) * 1.0
        self.ax_top.plot([base_x, link1_x], [base_y, link1_y], marker="o", color="#ff5733", linewidth=4)


        shoulder_x, shoulder_y = 0, 0
        elbow_x = shoulder_x + np.cos(np.radians(self.joint_values[1])) * 1.1
        elbow_y = shoulder_y + np.sin(np.radians(self.joint_values[1])) * 1.1
        wrist_x = elbow_x + np.cos(np.radians(self.joint_values[2]+self.joint_values[1])) * 1.0
        wrist_y = elbow_y + np.sin(np.radians(self.joint_values[2]+self.joint_values[1])) * 1.0
        end_x = wrist_x + np.cos(np.radians(self.joint_values[3]+self.joint_values[2]+self.joint_values[1])) * 0.8
        end_y = wrist_y + np.sin(np.radians(self.joint_values[3]+self.joint_values[2]+self.joint_values[1])) * 0.8

        self.ax_front.plot([shoulder_x, elbow_x, wrist_x, end_x],
                           [shoulder_y, elbow_y, wrist_y, end_y],
                           marker="o", color="#007acc", linewidth=4)


        self.canvas.draw()

    def update_loop(self):
        """Periodic update loop for visualization."""
        self.update_visualization()
        self.root.after(100, self.update_loop)

def main(args=None):
    rclpy.init(args=args)
    gui_node = gui_run()
    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
