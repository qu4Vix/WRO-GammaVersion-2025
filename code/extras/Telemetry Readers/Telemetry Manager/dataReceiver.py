import modules.receive_utils as recUtils

# ESP32 device properties
udp_adress = "192.168.144.14"
udp_port = 4210

# set up udp socket
receiver = recUtils.Receiver(udp_adress, udp_port, 0.5)