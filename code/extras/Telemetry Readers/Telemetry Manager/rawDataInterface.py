# Import Utils
import modules.receive_utils as recUtils
import modules.data_structures as structs

# Web socket setup
receive_adress = "localhost"
receive_port = 1236

receiver = recUtils.Receiver(receive_adress, receive_port)
print("Initialized")

packets = tuple(structs.dataPacket(i) for i in range(0 , structs.TOTAL_PACKETS))

# Main loop
while True:
    if (receiver.listen(4096)):
        data = list(receiver.data)
        pType = data[0]
        packets[pType].setDataPacket(data)
        print("Received data: " + str(packets[pType].decode()))