import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.rcParams.update({'font.size': 16})

color1 = [0, 0, 0.5]
color2 = [0.5, 0.5, 0.5]
linestyle1 = '-'
linestyle2 = '--'

def plotTimePosition3(time, position3):
    plt.gcf()
    plt.plot(time, position3[:,0], linewidth=4.0, linestyle=linestyle1, color=color1)
    plt.plot(time, position3[:,1], linewidth=4.0, linestyle=linestyle2, color=color1)
    plt.plot(time, position3[:,2], linewidth=2.0, linestyle=linestyle1, color=color2)
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.grid()
    plt.legend(['x','y','z'], loc='best');
