import numpy as np 
import matplotlib.pyplot as plt  


fix,ax = plt.subplots()

e1 = np.load('008_lidar_default_imu.npy')
e2 = np.load('016_lidar_default_imu.npy')
e3 = np.load('032_lidar_default_imu.npy')
e4 = np.load('064_lidar_default_imu.npy')

x = range(e1.size)

# ax.plot(x,e1)
# plt.title('Average Map Accuracy vs. Calculated Points \n (demo)')
# plt.xlabel('Points Calculated')
# plt.ylabel('Average Euclidean Error (m)')
# plt.show()
# ax.plot(x,e1,label='sigma=0.008 (default)')

# ax.plot(x,e2,label='sigma=0.016')
# ax.plot(x,e3,label='sigma=0.032')
# ax.plot(x,e4,label='sigma=0.064')

# # avg = e1[-1] + e2[-1] + e3[-1] + e4[-1]
# # avg /=4
# # print(avg)

# plt.title('Average Map Error vs. Calculated Points \n (changing VLP noise)')
# plt.xlabel('Points Calculated')
# plt.ylabel('Average Euclidean Error (m)')
# ax.legend()
# # plt.savefig('map_error_changing_vlp.png')
# plt.show()

# fig,ax = plt.subplots()

# e6 = np.load('01_imu_default_lidar.npy')
# e7 = np.load('02_imu_default_lidar.npy')
# e8 = np.load('04_imu_default_lidar.npy')

# avg = e1[-1] + e6[-1] + e7[-1] + e8[-1]
# avg /=4
# print(avg)

# ax.plot(x,e1,label='sigma=0.005 (default')
# ax.plot(x,e6,label='sigma=0.01')
# ax.plot(x,e7,label='sigma=0.02')
# ax.plot(x,e8,label='sigma=0.04')

# plt.title('Average Map Error vs. Calculated Points \n (changing IMU noise)')
# plt.xlabel('Points Calculated')
# plt.ylabel('Average Euclidean Error (m)')
# ax.legend()
# # plt.savefig('map_error_changing_imu.png')
# plt.show()

e9 = np.load('instantaneous_errors_defaults.npy')
x = range(e9.size)

fig,ax = plt.subplots()
ax.plot(x,e9)
plt.title('Instantaneous Map Error vs. Calculated Points')
plt.xlabel('Points Calculated')
plt.ylabel('Instantaneous Euclidean Error (m)')
plt.savefig('playpen_errors_instantaneous.png')
plt.show()


