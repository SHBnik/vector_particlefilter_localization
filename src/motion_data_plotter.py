from numpy import genfromtxt
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as stats
import math




PI = 3.1415926535897


my_data = genfromtxt('/home/shb/catkin_ws/src/artest/src/motion_data_-90deg.csv', delimiter=',')


delta_translation = np.array([])
delta_rotation = np.array([])

for line in my_data:
    trans = math.sqrt((line[0])**2 + (line[1])**2)
    delta_translation = np.append (delta_translation, float(trans))
    delta_rotation = np.append (delta_rotation, float(line[2]) * 180 / PI)



mu_trans = delta_translation.mean()
variance_trans = delta_translation.var()

mu_rot = delta_rotation.mean()
variance_rot = delta_rotation.var()

print(mu_trans,variance_trans)
print(mu_rot,variance_rot)




sigma1 = math.sqrt(variance_trans)
x1 = np.linspace(mu_trans - 3*sigma1, mu_trans + 3*sigma1, 100)

sigma2 = math.sqrt(variance_rot)
x2 = np.linspace(mu_rot - 3*sigma2, mu_rot + 3*sigma2, 100)

fig, (ax1, ax2) = plt.subplots(2, 1)
ax1.plot(x1, stats.norm.pdf(x1, mu_trans, sigma1))
ax1.set_title("translation")
ax2.plot(x2, stats.norm.pdf(x2, mu_rot, sigma2))
ax2.set_title("rotation")

plt.show()