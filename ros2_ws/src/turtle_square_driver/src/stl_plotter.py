#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

class STLPlotter(Node):
    def __init__(self):
        super().__init__('stl_plotter')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/stl_metrics',
            self.metrics_callback,
            10
        )
        self.time_data = []
        self.sat1_data = []
        self.sat2_data = []
        self.get_logger().info("STL Plotter Active. Waiting for /stl_metrics data...")

    def metrics_callback(self, msg):
        # Expecting msg.data = [current_sim_time, sat1, sat2]
        if len(msg.data) >= 3:
            self.time_data.append(msg.data[0])
            self.sat1_data.append(msg.data[1])
            self.sat2_data.append(msg.data[2])
            
            # Keep a rolling window of ~20 seconds (1250 ticks at 16ms/tick)
            if len(self.time_data) > 1250:
                self.time_data.pop(0)
                self.sat1_data.pop(0)
                self.sat2_data.pop(0)

# --- Threading and Live Plot Setup ---
plotter_node = None

def ros_spin_thread():
    rclpy.spin(plotter_node)

def update_plot(frame, ax1, ax2, line_s1, line_s2):
    if not plotter_node or not plotter_node.time_data:
        return line_s1, line_s2
    
    t = plotter_node.time_data
    line_s1.set_data(t, plotter_node.sat1_data)
    line_s2.set_data(t, plotter_node.sat2_data)
    
    x_max = t[-1]
    x_min = max(0, x_max - 20.0)
    
    # Scale X-axis, anchoring at 20 seconds minimum to prevent erratic zooming at startup
    ax1.set_xlim(x_min, max(20.0, x_max))
    ax2.set_xlim(x_min, max(20.0, x_max))
    
    return line_s1, line_s2

def main(args=None):
    global plotter_node
    rclpy.init(args=args)
    
    plotter_node = STLPlotter()
    
    thread = threading.Thread(target=ros_spin_thread, daemon=True)
    thread.start()
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 3), sharex=True)
    fig.suptitle('Real-Time STL')
    
    # Step 1: Inbounds Satisfaction
    line_s1, = ax1.plot([], [], color='blue', drawstyle='steps-post')
    ax1.set_ylabel('Step 1 Sat\n(In Bounds)')
    ax1.set_ylim(-0.2, 1.2)
    ax1.set_yticks([0, 1])
    ax1.grid(True, alpha=0.3)
    
    # Step 2: Recovery Satisfaction
    line_s2, = ax2.plot([], [], color='green', drawstyle='steps-post')
    ax2.set_ylabel('Step 2 Sat\n(Recovery)')
    ax2.set_ylim(-0.2, 1.2)
    ax2.set_yticks([0, 1])
    ax2.set_xlabel('Time (s)')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    ani = animation.FuncAnimation(fig, update_plot, fargs=(ax1, ax2, line_s1, line_s2), interval=50, blit=False)
    
    plt.show()
    
    plotter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()