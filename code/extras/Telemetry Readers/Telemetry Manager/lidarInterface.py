import time

from math import sin,cos,radians

from tkinter import *

import modules.receive_utils as recUtils
import modules.data_structures as structs

MAX_RANGE = 6000
CANVAS_SIZE = 700

root = Tk()
root.title("LIDAR display")
root.geometry(str(CANVAS_SIZE+10) + "x" + str(CANVAS_SIZE + 5))
root.resizable(False, False)

canvas = Canvas(width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white")
canvas.pack()
canvas.create_oval(CANVAS_SIZE /2 -4, CANVAS_SIZE/2 -4, CANVAS_SIZE/2 +4, CANVAS_SIZE/2 +4,fill="red")

lidarPoints = []

def real_to_canvas(radius):
    return float(radius) * (CANVAS_SIZE - 15) / MAX_RANGE

def drawPoint(r, theta):
    _r = real_to_canvas(r)
    _theta = float(theta)
    x = _r * sin(radians(_theta)) + CANVAS_SIZE/2
    y = -_r * cos(-radians(_theta)) + CANVAS_SIZE/2
    lidarPoints.append(canvas.create_oval(x-4, y-4, x+4, y+4, fill="lightblue"))

def manageInput(data):
    dStruct = structs.dataPacket(structs.packetNames["Lidar Distances"])
    dStruct.setDataPacket(data)
    array = dStruct.decode()["LidarDistances"]
    print(array)
    for i in range(0, len(lidarPoints)):
        canvas.delete(lidarPoints[i])
    for i in range(0, 360):
        _radius = array[i]
        if _radius != 0:
            drawPoint(_radius, i)

def timedReception():
    if (receiver.listen(721)):
        data = list(receiver.data)
        manageInput(data)
    root.after(500, timedReception)

# Web socket setup
receive_adress = "localhost"
receive_port = 1239

receiver = recUtils.Receiver(receive_adress, receive_port, 0.5)
print("Initialized")

timedReception()
# Main loop
root.mainloop()