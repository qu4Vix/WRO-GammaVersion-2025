# Time management
import time

# GUI
from tkinter import *

import math

class car():
    def __init__(self, canvas, width, height) -> None:
        self.canvas = canvas
        self.carWidth = width
        self.carHeight = height
        self.carCenter_x = 0
        self.carCenter_y = 0
        self.rotation = 0
        self.carBodyId = self.canvas.create_polygon(self.carCoords(), fill='red')
        self.carLidarId = self.canvas.create_oval(self.lidarCoords(), fill='black')
    def carCoords(self):
        # maybe you have to -degrees to rotate in the same direction as the car
        rad = math.radians(self.rotation)
        x1 = self.carCenter_x + math.cos(rad) * (-self.carWidth / 2) - math.sin(rad) * (-self.carHeight / 2)
        y1 = self.carCenter_y + math.sin(rad) * (-self.carWidth / 2) + math.cos(rad) * (-self.carHeight / 2)
        x2 = self.carCenter_x + math.cos(rad) * (self.carWidth / 2) - math.sin(rad) * (-self.carHeight / 2)
        y2 = self.carCenter_y + math.sin(rad) * (self.carWidth / 2) + math.cos(rad) * (-self.carHeight / 2)
        x3 = self.carCenter_x + math.cos(rad) * (self.carWidth / 2) - math.sin(rad) * (self.carHeight / 2)
        y3 = self.carCenter_y + math.sin(rad) * (self.carWidth / 2) + math.cos(rad) * (self.carHeight / 2)
        x4 = self.carCenter_x + math.cos(rad) * (-self.carWidth / 2) - math.sin(rad) * (self.carHeight / 2)
        y4 = self.carCenter_y + math.sin(rad) * (-self.carWidth / 2) + math.cos(rad) * (self.carHeight / 2)
        return x1, y1, x2, y2, x3, y3, x4, y4
    def lidarCoords(self):
        rad = math.radians(self.rotation)
        x1 = self.carCenter_x + math.sin(rad) * (self.carHeight / 2)
        y1 = self.carCenter_y - math.cos(rad) * (self.carHeight / 2)
        x2 = self.carCenter_x + math.cos(rad) * (self.carWidth / 2) - math.sin(rad) * (self.carHeight / 2)
        y2 = self.carCenter_y + math.sin(rad) * (self.carWidth / 2) + math.cos(rad) * (self.carHeight / 2)
        return x1, y1, x2, y2
    def moveCar(self, x, y, orientation=None):
        self.carCenter_x = int(x)
        self.carCenter_y = int(y)
        self.rotateCar(int(orientation) if orientation else self.rotation)
    def rotateCar(self, degrees):
        # maybe you have to -degrees to rotate in the same direction as the car
        self.rotation = degrees
        self.canvas.coords(self.carBodyId, self.carCoords())
        self.canvas.coords(self.carLidarId, self.lidarCoords())

# work out the global coords from local coords