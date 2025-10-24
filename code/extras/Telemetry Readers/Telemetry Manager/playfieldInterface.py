# Import time
#import time

# GUI
from tkinter import *

import modules.receive_utils as recUtils
import modules.playfield_utils as fieldUtils
import modules.data_structures as structs

# Web socket setup
receive_adress = "localhost"
receive_port = 1235

receiver = recUtils.Receiver(receive_adress, receive_port, 0.5)

CANVAS_SIZE = 600
FIELD_SIZE = 3000


root = Tk()
root.title("Playfield display")
root.geometry(str(CANVAS_SIZE+10) + "x" + str(CANVAS_SIZE + 5))

canvas = Canvas(width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white")
canvas.pack()

# Each pixel in the canvas is 5mm in the real game mat

# Sections
canvas.create_line(0, 200, 600, 200, fill="lightgrey")
canvas.create_line(0, 400, 600, 400, fill="lightgrey")
canvas.create_line(200, 0, 200, 600, fill="lightgrey")
canvas.create_line(400,0, 400, 600, fill="lightgrey")

# Start squares (right)
canvas.create_line(520, 200, 520, 400, fill="lightgrey")
canvas.create_line(480, 200, 480, 400, fill="lightgrey")
canvas.create_line(400, 300, 600, 300, fill="lightgrey")
# Start squares (bottom)
canvas.create_line(200, 520, 400, 520, fill="lightgrey")
canvas.create_line(200, 480, 400, 480, fill="lightgrey")
canvas.create_line(300, 400, 300, 600, fill="lightgrey")
# Start squares (left)
canvas.create_line(80, 200, 80, 400, fill="lightgrey")
canvas.create_line(120, 200, 120, 400, fill="lightgrey")
canvas.create_line(0, 300, 200, 300, fill="lightgrey")
# Start squares (top)
canvas.create_line(200, 80, 400, 80, fill="lightgrey")
canvas.create_line(200, 120, 400, 120, fill="lightgrey")
canvas.create_line(300, 0, 300, 200, fill="lightgrey")

# Middle text
canvas.create_rectangle(220, 240, 380, 360, fill="blue", outline="")
canvas.create_rectangle(240, 220, 360, 380, fill="blue", outline="")
canvas.create_oval(220, 220, 260, 260, fill="blue", outline="")
canvas.create_oval(340, 340, 380, 380, fill="blue", outline="")
canvas.create_oval(220, 340, 260, 380, fill="blue", outline="")
canvas.create_oval(340, 220, 380, 260, fill="blue", outline="")
canvas.create_text(300, 300, text="Gamma\nVersion", fill="white", font="Arial 15 bold")

# Car
car = fieldUtils.car(canvas, 50, 31)
car.moveCar(300, 100)

def real_to_canvas(radius):
    return float(radius) * (CANVAS_SIZE - 15) / FIELD_SIZE

dataPacket = structs.dataPacket(structs.packetNames["General Information"])

def handleReception():
    if (receiver.listen(41)):
        #print("Received data:" + receiver.data.decode())
        dataPacket.setDataPacket(receiver.data)
        data = dataPacket.decode()
        car.moveCar(data["PosicionX"]/5, data["PosicionY"]/5)
    root.after(100, handleReception)

handleReception()
# Main loop
root.mainloop()