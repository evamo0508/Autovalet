import numpy as np
import matplotlib.pyplot as plt

def main():
	print "Plotting errors"
	average = np.load("0.04imu_aError.npy")
	yaw = np.load("0.04imu_yError.npy")
	pos = np.load("0.04imu_pError.npy")

	plt.figure(1)
	plt.scatter(np.arange(average.shape[0]), average)
	plt.title("Running average")
	plt.figure(2)
	plt.scatter(np.arange(yaw.shape[0]), yaw)
	plt.title("Yaw error")
	plt.figure(3)
	plt.scatter(np.arange(pos.shape[0]), pos)
	plt.title("Instantaneous Errors")
	plt.show()

if __name__ == '__main__':
	main()