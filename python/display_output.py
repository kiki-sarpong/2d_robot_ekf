import matplotlib.pyplot as plt
import numpy as np
import os

class DisplayOutput:
    """
    This class is used to load values from a txt file and display them in a GUI
    """
    def __init__(self) -> None:
        ground_truth_filepath = ''.join([os.getcwd(), "/ground_truth.txt"])
        ekf_output_filepath = ''.join([os.getcwd(), "/ekf_output.txt"])
        lidar_output_filepath = ''.join([os.getcwd(), "/lidar_output.txt"])
        number_of_points = 121
        self.ekf_points = self.read_file(np.zeros((number_of_points, 6)), ekf_output_filepath)
        self.gt_points = self.read_file(np.zeros((number_of_points, 4)), ground_truth_filepath)
        self.lidar_points = self.read_file(np.zeros((number_of_points, 2)), lidar_output_filepath)

    def read_file(self, points:np.ndarray, file_path:str) -> np.ndarray:
        """
        This method reads data from a txt file into a numpy array
        """
        with open(file_path, "r") as f:
            for i, line in enumerate(f):
                l = line.strip('\n').split(" ")
                points[i] = [float(n) for n in l]
        return points
        
    def display_output(self) -> None: 
        """
        This method displays the data with matplotlib
        """
        _, axs= plt.subplots(2, 2, figsize=(12, 8))
        # fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)

        axs[0, 0].set_ylabel("Y points")
        axs[0, 0].set_xlabel("X points")
        axs[0, 0].set_title("Extended Kalman Filter X/Y Comparison (Ground Truth vs EKF)")
        # X and Y postition comparision
        axs[0, 0].plot(self.gt_points[:,0], self.gt_points[:,1], label="Ground truth x/y points")
        axs[0, 0].plot(self.ekf_points[:,0], self.ekf_points[:,1], label="EKF x/y points")
        axs[0, 0].plot(self.lidar_points[:,0], self.lidar_points[:,1], "o", label="lidar points", markersize=2)
        axs[0, 0].legend(fontsize="small")

        # Orientation plots
        axs[0, 1].set_xlabel("Time Steps")
        axs[0, 1].set_ylabel("Orientation (radians)")
        axs[0, 1].set_title("Orientation Comparison (Ground Truth vs EKF)")
        axs[0, 1].plot(self.gt_points[:,2], label="Ground truth orientation ")
        axs[0, 1].plot(self.ekf_points[:,2],  label="EKF orientation")
        axs[0, 1].legend(fontsize="small")

        # Velocity 
        axs[1, 0].set_xlabel("Time Steps")
        axs[1, 0].set_ylabel("Velocity (m/s)")
        axs[1, 0].set_title("EKF velocity")
        velocity = np.linalg.norm([self.ekf_points[:,3], self.ekf_points[:,4]], axis=0)
        axs[1, 0].plot(self.gt_points[:,3], label="Ground truth velocity")
        axs[1, 0].plot(velocity, label="EKF velocity")
        axs[1, 0].legend(fontsize="small")

        # Angular velocity
        axs[1, 1].set_xlabel("Time Steps")
        axs[1, 1].set_ylabel("Angular velocity")
        axs[1, 1].set_title("EKF Angular velocity")
        axs[1, 1].plot(self.ekf_points[:,5], label="EKF angular velocity")
        axs[1, 1].legend(fontsize="small")

        plt.tight_layout()  # Adjust layout to avoid overlap
        plt.show()


def main():
    display = DisplayOutput()
    try:
        display.display_output()
    except KeyboardInterrupt as e:
        print("Successfully exited program.")


if __name__ == '__main__':
    main()