#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt
from collections import deque

class SignalPlotter(Node):
    def __init__(self):
        super().__init__('signal_plotter')
        
        # Variables to store noisy and filtered data
        self.noisy = deque(maxlen=100)  # Store up to 100 points of noisy data
        self.filtered = deque(maxlen=100)  # Store up to 100 points of filtered data
        self.alpha = 0.01  # Smoothing factor for EMA
        self.estimate = None  # Initial estimate for EMA
        
        # Subscriber to the /whycon/poses topic
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 10)

        # Initialize the plot
        plt.ion()  # Enable interactive mode
        self.figure, self.ax = plt.subplots(figsize=(10, 6))
        self.line_noisy, = self.ax.plot([], [], label='Noisy Signal', color='red')
        self.line_filtered, = self.ax.plot([], [], label='Filtered Signal', color='blue')
        self.ax.set_xlabel('Time Steps')
        self.ax.set_ylabel('Signal Value')
        self.ax.set_title('Noisy vs Filtered Signal')
        self.ax.legend()

    def update(self, value):
        """
        Applies an Exponential Moving Average (EMA) filter to the input value.
        """
        if self.estimate is None:
            self.estimate = value  # Initialize with the first value
        else:
            self.estimate = self.alpha * value + (1 - self.alpha) * self.estimate
        return self.estimate

    def whycon_callback(self, msg):
        """
        Callback for the /whycon/poses topic.
        Extracts the noisy signal and applies the EMA filter.
        """
        # Extract the noisy z-position (altitude)
        noisy_data = msg.poses[0].position.z
        self.noisy.append(noisy_data)
        
        # Filter the noisy data
        filtered_data = self.update(noisy_data)
        self.filtered.append(filtered_data)
        
        # Update the plot
        self.plot_signals()

    def plot_signals(self):
        """
        Updates the plot with the latest noisy and filtered signals.
        """
        # Update the data for both plots
        self.line_noisy.set_data(range(len(self.noisy)), list(self.noisy))
        self.line_filtered.set_data(range(len(self.filtered)), list(self.filtered))
        
        # Adjust plot limits
        self.ax.relim()
        self.ax.autoscale_view()
        
        # Redraw the plot
        plt.draw()
        plt.pause(0.01)

    def main_loop(self):
        """
        Main loop for running the node and plotting.
        """
        rclpy.spin(self)  # Keep the node running
        plt.ioff()  # Turn off interactive mode
        plt.show()  # Show the final plot


def main(args=None):
    """
    Main entry point for the script.
    """
    rclpy.init(args=args)
    node = SignalPlotter()
    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
