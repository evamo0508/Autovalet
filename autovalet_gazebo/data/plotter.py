import numpy as np 
import matplotlib.pyplot as plt  

fix,ax = plt.subplots()

# Running average error for lidar noise variation
e1 = np.load('0.008_aError.npy')
# e2 = np.load('0.016_aError.npy')
# e3 = np.load('0.032_aError.npy')
# e4 = np.load('0.064_aError.npy')

# Instantaneous error for lidar noise variation
# e1 = np.load('0.008_pError.npy')
# e2 = np.load('0.016_pError.npy')
# e3 = np.load('0.032_pError.npy')
# e4 = np.load('0.064_pError.npy')

# Instantaneous Yaw error for lidar noise variation
# e1 = np.load('0.008_yError.npy')
# e2 = np.load('0.016_yError.npy')
# e3 = np.load('0.032_yError.npy')
# e4 = np.load('0.064_yError.npy')


## Instantaneous Yaw error for IMU noise variation
# e1 = np.load('0.008_yError.npy')
# e2 = np.load('0.01imu_yError.npy')
# e3 = np.load('0.02imu_yError.npy')
# e4 = np.load('0.04imu_yError.npy')

# e1[e1>5] -= 2*np.pi
# e2[e2>5] -= 2*np.pi
# e3[e3>5] -= 2*np.pi
# e4[e4>5] -= 2*np.pi

# e1 = np.abs(e1[e1<5])
# e2 = np.abs(e2[e2<5])
# e3 = np.abs(e3[e3<5])
# e4 = np.abs(e4[e4<5])

# e1 = e1*180/3.14
# e2 = e2*180/3.14
# e3 = e3*180/3.14
# e4 = e4*180/3.14
##================================================#

# IMU pose error
# e1 = np.load('0.008_pError.npy')
# e2 = np.load('0.01imu_pError.npy')
# e3 = np.load('0.02imu_pError.npy')
# e4 = np.load('0.04imu_pError.npy')

#Avg error IMU
# e1 = np.load('0.008_aError.npy')
# e2 = np.load('0.01imu_aError.npy')
# e3 = np.load('0.02imu_aError.npy')
# e4 = np.load('0.04imu_aError.npy')

# x = range(max(e1.shape[0], e2.shape[0], e3.shape[0], e4.shape[0]))
x = range(e1.shape[0])

# print(float(e1[e1<0.5].shape[0])/float(e1.shape[0]))
# print(float(e2[e2<0.5].shape[0])/float(e2.shape[0]))
# print(float(e3[e3<0.5].shape[0])/float(e3.shape[0]))
# print(float(e4[e4<0.5].shape[0])/float(e4.shape[0]))

# print(e1[-1], e2[-1], e3[-1], e4[-1])
# print(np.mean(e1), np.mean(e2), np.mean(e3), np.mean(e4))

ax.plot(x[0:e1.shape[0]],e1,label='svd_test_run')
# ax.plot(x[0:e2.shape[0]],e2,label='sigma=0.01')
# ax.plot(x[0:e3.shape[0]],e3,label='sigma=0.02')
# ax.plot(x[0:e4.shape[0]],e4,label='sigma=0.04')

plt.title('(Running)Average euclidean error vs. Sample point \n Varying IMU noise')
plt.xlabel('Error Sample point/ Dist moved by robot')
plt.ylabel('Average position Error (m)')

ax.legend(loc="lower right")
plt.show()
