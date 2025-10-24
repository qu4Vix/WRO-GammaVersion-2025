#
# Simple module to define the receiver class
# The receiver is innitialised with an adress, port and timeout. It sets up a socket with such parameters.
# The receiver's listen method receives the given buffer size from the socket and stores it in the data array

# Time management
import time

# Web socket
import socket

# Main class
class Receiver():
    def __init__(self, adress, port, timeout=0) -> None:
        self.recSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recSocket.bind((adress, port))
        if (timeout): #if timeout is non zero assign it to the socket
            self.recSocket.settimeout(timeout)
        self.data = []
        self.addr = None
    def listen(self, bufSize):
        try:
            self.data, self.addr = self.recSocket.recvfrom(bufSize) #receive the expected bytes
        except socket.timeout:
            #print("timeout")
            self.data, self.addr = None, None
        if not self.data: #if no data was received, return false
            return False
        return True #otherwise return true