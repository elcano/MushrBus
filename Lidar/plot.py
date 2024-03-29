import os
import ydlidar
import sys
from matplotlib.patches import Arc
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import csv

RMAX = 32.0
NUM_SHOW = 2   # how many data in the file do you wish to see?

# Specify the data file path
data_file = 'data/12202021/lidar_data_noisy_box_3m.csv'
f = open(data_file, 'r', newline='')
reader = csv.reader(f)

def plot():
        dataset = []
        angle = []
        ran = []
        intensity = []
        temp = 0
        for i, row in enumerate(reader):
                if i == 0:
                        continue
                if i == 1:
                        temp = float(row[0])
                if temp != float(row[0]):
                        dataset.append([angle, ran, intensity])
                        angle = []
                        ran = []
                        intensity = []
                        temp = float(row[0])
                angle.append(float(row[1]))
                ran.append(float(row[2]))
                intensity.append(float(row[3]))
        dataset.append([angle, ran, intensity])
        print(len(dataset))

        '''
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.scatter(x = angle, y = ran)
        plt.show()
        '''

        for i, data in enumerate(dataset):
                if i >= NUM_SHOW:
                        break
                # fig = plt.figure()
                # fig.canvas.set_window_title('YDLidar LIDAR Monitor')
                lidar_polar = plt.subplot(polar=True)
                lidar_polar = plt.subplot(polar=True)
                lidar_polar.autoscale_view(True,True,True)
                lidar_polar.set_rmax(RMAX)
                lidar_polar.grid(True)

                lidar_polar.scatter(data[0], data[1], c=data[2], cmap='hsv', alpha=0.95)
                # lidar_polar.scatter(angle, ran, cmap='hsv', alpha=0.95)



                # lidar_polar.clear()
                # lidar_polar.scatter(angle, ran, alpha=0.95)
                plt.show()

if __name__ == "__main__":
        plot()