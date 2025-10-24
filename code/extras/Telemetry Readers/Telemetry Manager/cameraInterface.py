# GUI
from tkinter import *

import modules.receive_utils as recUtils

root = Tk()
root.title("Camera display")
root.geometry("610x605")
root.resizable(False, False)

canvas = Canvas(width=600, height=600, bg="white")
canvas.pack()

blocks = []

def drawBlock(x, y, width, height, color):
    _x, _y, _width, _height, _color = int(x), int(y), int(width), int(height), color.strip()
    blocks.append(canvas.create_rectangle(_x, _y, _x + _width, _y + _height, fill=_color))

def clearBlocks():
    for block in blocks:
        canvas.delete(blocks[blocks.index(block)])
    blocks.clear()

# Web socket setup
receive_adress = "localhost"
receive_port = 1238

receiver = recUtils.Receiver(receive_adress, receive_port, 1)

# Main loop
while True:
    root.update()
    root.update_idletasks()
    if (receiver.listen(1024)):
        clearBlocks()
        #drawBlock(*tuple(receiver.data.decode().split(",")))
        print(tuple(receiver.data.decode().split(",")))
    else:
        clearBlocks()