#
# Program made by the Gamma Version Team
#

# Time management
#import time

# Utils
import modules.receive_utils as recUtils
import modules.velplotter_utils as velUtils

# Web socket setup
receive_adress = "localhost"
receive_port = 1237

receiver = recUtils.Receiver(receive_adress, receive_port, 1)

# Main loop
while True:
    if (receiver.listen(1024)):
        velUtils.updatePlot(*tuple(receiver.data.decode().split(",")))
        print("Received data:" + receiver.data.decode())