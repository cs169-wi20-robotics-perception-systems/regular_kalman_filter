#! /usr/bin/python

import numpy as np
import matplotlib.pyplot as plt


# Plot pose estimates

truth_y = np.array([0.0, 0.97])
truth_x = np.array([0.0, 5.0])

pose_y = np.array([0.0371670723, 0.0741047263, 0.1130697727, 0.1516419649, 0.1885138154,
                    0.2277396917, 0.2666388154, 0.3017787337, 0.3367234468, 0.3737272024,
                    0.4085059166, 0.4451829195, 0.4786226749, 0.5135993958, 0.5528253913,
                    0.5895680785, 0.6303626299, 0.6650782227, 0.7018842101, 0.7392466664,
                    0.7723279595, 0.8075333238, 0.8423147798, 0.8773571849, 0.9100453258,
                    0.9451506734, 0.9822514653, 1.0172943473])
pose_x = np.array([0.2, 0.4, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9, 2.0, 2.2, 2.4, 2.6, 2.8,
                    3.0, 3.2, 3.3, 3.5, 3.7, 3.9, 4.0, 4.2, 4.4, 4.5, 4.7, 4.9, 5.0])

cmd_vel_and_scan_y = np.array([0.0069063758, 0.0224866596, 0.0392632361, 0.0563569912,
                    0.0710049918, 0.1052211415, 0.1215715902, 0.13855909, 0.1544443418,
                    0.1866494762, 0.2541679025, 0.270668636, 0.2855994355, 0.3025596476,
                    0.3323087616, 0.3502405539, 0.36657198, 0.5773577027, 0.5983799394,
                    0.6135492945, 0.6475258509, 0.6646562899, 0.9092609925, 0.9306723089,
                    0.9642415363])
cmd_vel_and_scan_x = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.3, 1.4, 1.5,
                    1.6, 1.7, 1.8, 1.9, 3.0, 3.1, 3.2, 3.3, 3.4, 4.8, 4.9, 5.0])

pose_and_scan_y = np.array([0.034465197, 0.0711022387, 0.108201381, 0.1435875835, 0.1819678087,
                    0.2187682631, 0.2555090529, 0.2884170581, 0.3279861816, 0.3634070365,
                    0.3992276958, 0.4354266932, 0.4764365951, 0.5167071451, 0.550086541,
                    0.5913849074, 0.6297048432, 0.6634942484, 0.6977324711, 0.7312202089,
                    0.7711996471, 0.8076793154, 0.8447035526, 0.8802801362, 0.9146763987,
                    0.9545285517, 0.9736661013])
pose_and_scan_x = np.array([0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6,
                    2.8, 3.0, 3.2, 3.3, 3.5, 3.7, 3.9, 4.1, 4.2, 4.4, 4.6, 4.8, 5.0])

cmd_vel_and_rgbd_y = np.array([0.2651382885, 0.0828772276, 0.5878363918, 0.80620975, 0.9628583029])
cmd_vel_and_rgbd_x = np.array([0.8, 1.5, 3.3, 3.5, 4.7])

pose_and_rgbd_y = np.array([0.1172657914, 0.642858991, 0.9131202756, 1.0997401157, 0.9991100204])
pose_and_rgbd_x = np.array([0.1, 2.1, 3.0, 4.0, 4.9])

plt.title("Robot Moving Forward 1 m")
plt.ylabel("state")
plt.xlabel("time (sec)")

plt.plot(truth_x, truth_y, label='ground truth')
plt.plot(pose_x, pose_y, label='pose')
plt.plot(cmd_vel_and_scan_x, cmd_vel_and_scan_y, label='cmd_vel and scan')
plt.plot(pose_and_scan_x, pose_and_scan_y, label='pose and scan')
plt.plot(cmd_vel_and_rgbd_x, cmd_vel_and_rgbd_y, label='cmd_vel and rgbd')
plt.plot(pose_and_rgbd_x, pose_and_rgbd_y, label='pose and rgbd')

plt.legend()
plt.show()


# Plot error in pose estimates

readings = ['pose', 'cmd_vel\n& scan', 'pose\n& scan', 'cmd_vel\n& rgbd', 'pose\n& rgbd']
errors = [-0.04729, 0.00575847, -0.0036661, 0.0071417, -0.02911]

fig, ax = plt.subplots()
ax.bar(readings, errors)

ax.set_title('Robot Moving Forward 1 m')
ax.set_ylabel('Errors (m)')
ax.set_yticks(np.arange(-0.05, 0.05, 0.01))
plt.show()
