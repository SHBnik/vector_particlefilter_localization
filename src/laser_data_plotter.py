import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as stats
import math

csv_path = '/home/shb/20.csv'



a = np.array([])


_file = open(csv_path)
for line in _file:
    # print(line)
    a = np.append (a, float(line))

_file.close()



mu = a.mean()
variance = a.var()
print(mu)
print(variance)

print(a.min())
print(a.max())



sigma = math.sqrt(variance)
x = np.linspace(mu - 3*sigma, mu + 3*sigma, 100)
plt.plot(x, stats.norm.pdf(x, mu, sigma))
plt.show()