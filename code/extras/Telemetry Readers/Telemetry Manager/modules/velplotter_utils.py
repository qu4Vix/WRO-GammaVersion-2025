# Plotting libraries
import matplotlib.pyplot as plt

speed_values = []
time_values = []

max_plot_values = 100

fig, ax = plt.subplots()
line, = ax.plot(time_values, speed_values)

def updatePlot(newTime, newSpeed):
    _newTime = int(newTime)
    _newSpeed = int(newSpeed)
    if (len(speed_values) == max_plot_values):
        speed_values.pop(0)
        time_values.pop(0)
    speed_values.append(_newSpeed)
    time_values.append(_newTime)
    line.set_data(time_values, speed_values)
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    plt.pause(0.005)