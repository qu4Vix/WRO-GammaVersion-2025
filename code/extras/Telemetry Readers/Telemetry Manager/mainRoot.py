#
# Main telemetry program
# Runs the root interface, receives data from the ESP32, opens the child processess and sends them the data
# 
# Gamma Version 2025
#


# Time management
import time

# GUI
from tkinter import Tk

# Closing processes
import atexit
import os

# Import utils
import modules.dataManager_utils as manUtils
import modules.receive_utils as recUtils
import modules.data_structures as struct

# Receiver deivce properties
udp_adress = "192.168.1.18"
udp_port = 5007

# Set up udp socket
receiver = recUtils.Receiver(udp_adress, udp_port, 2)

# GUI setup (Set the name, size and not resizable)
root = Tk()
root.title("Telemetry manager")
root.geometry("350x310")
root.resizable(False, False)

# Function to close the process
def cleanUp():
    os._exit(0)

# When closing the interface, kill the program
root.protocol("WM_DELETE_WINDOW", cleanUp)
atexit.register(cleanUp)

# Data manager
names = struct.packetNames
dataManager = manUtils.dataManager(receiver)

# Interface manager definitions (creates the objects and places a button to run the process at the xy location stated)
playfield = manUtils.interfaceManager(1235, 20, "Playfield Display", "playfieldInterface.py", [names["General Information"]], dataManager)
rawdata = manUtils.interfaceManager(1236, 70, "Raw Data Display", "rawDataInterface.py", [i for i in range(0,len(names))], dataManager)
velocity = manUtils.interfaceManager(1237, 120, "Velocity Plotter", "velocityInterface.py", ["time", "speed"], dataManager)
camera = manUtils.interfaceManager(1238, 170, "Camera Display", "cameraInterface.py", ["time"], dataManager)
lidar = manUtils.interfaceManager(1239, 220, "LIDAR Points Display", "lidarInterface.py", [names["Lidar Distances"]], dataManager)
saveFile = manUtils.saveButton(150, 260)

# Function to handle the reception of the data and send it to the various interfaces, executed in a loop
def handleReception():
    if receiver.listen(1000000):
        dataManager.receiveData()
        #saveFile.addDataString(receiver.data)
        rawdata.sendPackets()
        lidar.sendPackets()
        playfield.sendPackets()
    root.after(100, handleReception)

# Execute the loop
handleReception()
# Main loop
root.mainloop()