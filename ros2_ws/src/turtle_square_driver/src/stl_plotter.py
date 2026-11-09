#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time

class STLPlotter(Node):
    def __init__(self):
        super().__init__('stl_plotter')
        self.subscription = self.create_subscription(
            Float32MultiArray, '/stl_metrics', self.metrics_callback, 10)
        self.num_formulas = 0
        self.time_data = []
        self.sat_data = [] # Will become a list of lists once we know how many formulas exist
        self.get_logger().info("Waiting for first /stl_metrics data to build dynamic plots...")

    def metrics_callback(self, msg):
        # On first message, initialize the data arrays based on message size
        if self.num_formulas == 0:
            self.num_formulas = len(msg.data) - 1  # Total length minus the time index
            self.sat_data = [[] for _ in range(self.num_formulas)]
            self.get_logger().info(f"Detected {self.num_formulas} STL formulas. Launching plots.")

        self.time_data.append(msg.data[0])
        for i in range(self.num_formulas):
            self.sat_data[i].append(msg.data[i+1])
            
        # Keep a rolling window of ~20 seconds
        if len(self.time_data) > 1250:
            self.time_data.pop(0)
            for i in range(self.num_formulas):
                self.sat_data[i].pop(0)

# --- Threading and Live Plot Setup ---
plotter_node = None

def ros_spin_thread():
    rclpy.spin(plotter_node)

def update_plot(frame, axes, lines):
    if not plotter_node or plotter_node.num_formulas == 0 or not plotter_node.time_data:
        return lines
    
    t = plotter_node.time_data
    x_max = t[-1]
    x_min = max(0, x_max - 20.0)

    for i in range(plotter_node.num_formulas):
        lines[i].set_data(t, plotter_node.sat_data[i])
        axes[i].set_xlim(x_min, max(20.0, x_max))
    
    return lines

def main(args=None):
    global plotter_node
    rclpy.init(args=args)
    
    plotter_node = STLPlotter()
    thread = threading.Thread(target=ros_spin_thread, daemon=True)
    thread.start()
    
    # Block main thread until the first message arrives to determine layout
    while plotter_node.num_formulas == 0:
        time.sleep(0.1)
        
    num_plots = plotter_node.num_formulas
    fig, axes = plt.subplots(num_plots, 1, figsize=(10, 1 * num_plots), sharex=True)
    fig.suptitle('Real-Time STL')
    
    # Force axes to be a list even if there's only 1 formula
    if num_plots == 1:
        axes = [axes]
        
    lines = []
    colors = ['blue', 'green', 'purple', 'orange', 'red', 'cyan', 'yellow', 'magenta', 'black']
    
    for i in range(num_plots):
        line, = axes[i].plot([], [], color=colors[i % len(colors)], drawstyle='steps-post')
        lines.append(line)
        axes[i].set_ylabel(f'phi{i+1} Sat')
        axes[i].set_ylim(-0.2, 1.2)
        axes[i].set_yticks([0, 1])
        axes[i].grid(True, alpha=0.3)
        
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    
    ani = animation.FuncAnimation(fig, update_plot, fargs=(axes, lines), interval=50, blit=False)
    plt.show()
    
    plotter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()