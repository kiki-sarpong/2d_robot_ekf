import matplotlib.pyplot as plt
import numpy as np
import os

class DisplayOutput:
    def __init__(self) -> None:
        self.ground_truth_filepath = ''.join([os.getcwd(), "/ground_truth.txt"])
        self.ekf_output_filepath = ''.join([os.getcwd(), "/ekf_output.txt"])
        number_of_points = 121
        self.gt_points = np.zeros((number_of_points, 3))
        self.ekf_points = np.zeros((number_of_points, 6))
        self.read_file()

    def read_file(self) -> None:
        with open(self.ground_truth_filepath, "r") as f:
            for i, line in enumerate(f):
                l = line.strip('\n').split(" ")
                self.gt_points[i] = [float(n) for n in l]

        with open(self.ekf_output_filepath, "r") as f:
            for i, line in enumerate(f):
                l = line.strip('\n').split(" ")
                self.ekf_points[i] = [float(n) for n in l]

        
    def display_output(self) -> None: 
        _, axs= plt.subplots(2, 2, figsize=(12, 8))
        axs[0, 0].set_ylabel("Y points")
        axs[0, 0].set_xlabel("X points")
        axs[0, 0].set_title("Extended Kalman Filter")
        # X and Y postition comparision
        axs[0, 0].plot(self.gt_points[:,0], self.gt_points[:,1], label="Ground truth x/y points")
        axs[0, 0].plot(self.ekf_points[:,0], self.ekf_points[:,1], label="EKF x/y points")
        axs[0, 0].legend()

        # Orientation plots
        axs[0, 1].set_xlabel("Time Steps")
        axs[0, 1].set_ylabel("Orientation (radians)")
        axs[0, 1].set_title("Orientation Comparison (Ground Truth vs EKF)")
        axs[0, 1].plot(self.gt_points[:,2], label="Ground truth orientation ")
        axs[0, 1].plot(self.ekf_points[:,2],  label="EKF orientation")
        axs[0, 1].legend()

        # Velocity 
        axs[1, 0].set_xlabel("Time Steps")
        axs[1, 0].set_ylabel("Velocity")
        axs[1, 0].set_title("EKF velocity")
        velocity = np.linalg.norm([self.ekf_points[:,3], self.ekf_points[:,4]], axis=0)
        axs[1, 0].plot(velocity,  label="EKF velocity")
        axs[1, 0].legend()

        # Angular velocity
        axs[1, 1].set_xlabel("Time Steps")
        axs[1, 1].set_ylabel("Angular velocity")
        axs[1, 1].set_title("EKF Angular velocity")
        axs[1, 1].plot(self.ekf_points[:,5])
        axs[1, 1].legend()

        plt.tight_layout()  # Adjust layout to avoid overlap
        plt.show()


def main():
    display = DisplayOutput()
    display.display_output()

if __name__ == '__main__':
    main()