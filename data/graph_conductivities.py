import matplotlib.pyplot as plt
import numpy as np
import os

path = ''
path += os.path.dirname(__file__)

file = open(path + '/sample_conductivities_data.txt', 'r')
iteration_cnt = (int)(file.readline().strip('\n'))
n = 9

data = np.zeros((iteration_cnt, n, n))

t = 0
i = 0
for line in file:
    line = line.strip('\n')
    line = line.rstrip()
    if line:
        converted_line = []
        split_line = line.split(sep = ' ')
        for nbr in split_line:
            converted_line.append((float)(nbr))
        data[t][i] = converted_line
        i += 1
    if i >= n:
        i = 0
        t += 1

#fig, axs = plt.subplots(n, n)

for i in range(n):
    for j in range(n):
        y_values = []
        for t in range(iteration_cnt):
            y_values.append(data[t][i][j])
        plt.plot(y_values)

plt.show()

