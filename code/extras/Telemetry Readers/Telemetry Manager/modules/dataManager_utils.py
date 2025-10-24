# Time management
import time

# GUI
from tkinter import *
from tkinter.filedialog import asksaveasfile

# Web socket
import socket

# Process management
import subprocess

# Data structures
import modules.data_structures as struct

class dataManager():
    def __init__(self, receiver):
        self.receiver = receiver
        self.dataPackets = tuple(struct.dataPacket(i) for i in range(0, struct.TOTAL_PACKETS))
        self.newPackets = []
    def receiveData(self):
        self.newPackets = []
        try:
            data = self.receiver.data
            packetType = data[0]
            self.newPackets.append(packetType)
            self.dataPackets[packetType].setDataPacket(data)
        except Exception:
            pass
    def requestPackets(self, packetRequests):
        out = []
        for i in range(0, len(packetRequests)):
            requestedIndex = packetRequests[i]
            if requestedIndex in self.newPackets:
                out += self.dataPackets[requestedIndex].data
        if out == []:
            return False
        return out
        

# Main class
class interfaceManager():
    childProcesses = []
    def __init__(self, port, y0, buttonName, programName, packetTypes, dataManager) -> None:
        # Data exchange
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.clientIp = "localhost"
        self.clientPort = port
        # GUI
        self.runButton = Button(text=buttonName, command=self.runInterface, width=30, bg="lightgrey", font="Arial 10 bold")
        self.closeButton = Button(text="X", command=self.closeInterface, width=3, bg="lightgrey", font="Arial 10 bold", fg="red")
        self.runButton.place(x=20, y=y0)
        self.closeButton.place(x=295, y=y0)
        # Other variables
        self.program = programName
        self.process = None
        self.active = False
        self.dataMangager = dataManager
        self.packetTypes = packetTypes
    def runInterface(self):
        if (not self.process):
            print(self.program + " running")
            self.process = subprocess.Popen(["python", self.program])
            interfaceManager.childProcesses.append(self.process)
            self.active = True
            print(interfaceManager.childProcesses)
            time.sleep(0.5)
    def closeInterface(self):
        if (self.process):
            interfaceManager.childProcesses.remove(self.process)
            self.process.kill()
            self.process = None
            self.active = False
            print(self.program + " killed")
    def sendBuffer(self, data):
        self.client.sendto(bytes(data), (self.clientIp, self.clientPort))
    def sendPackets(self):
        if (self.active):
            dataAvailable = self.dataMangager.requestPackets(self.packetTypes)
            if dataAvailable != False:
                self.sendBuffer(dataAvailable)

class saveButton():
    def __init__(self, x, y):
        self.SAVE_FILE_ICON = PhotoImage(file= r"30pxSaveIcon.png")         # Button image
        self.runButton = Button(image=self.SAVE_FILE_ICON, command=self.saveFile, width=30, bg="lightgrey", font="Arial 10 bold")   # Button config
        self.runButton.place(x=x, y=y)          # Button placement
        self.dataString = "Telemetry data:"     # Button text
    def saveFile(self):
        self.file = asksaveasfile(defaultextension= ".csv")
        self.file.write(self.dataString)
        self.file.close()
        self.file = None
        self.dataString = ""
    def addDataString(self, string):
        self.dataString += "\n" + string