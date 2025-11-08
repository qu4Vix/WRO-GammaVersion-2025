import socket

LOCAL_UDP_IP = "192.168.1.160"
SHARED_UDP_PORT = 5007
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP
sock.bind((LOCAL_UDP_IP, SHARED_UDP_PORT))
#def decodeBytes(data):
#    index = 0
#    decoded = []
#    if len(data) > 1:
#        cab = data[0]
#        if cab == 4:
#            for i in range(1,719, 2):
#                d = data[i]<<8 | (data[i+1])
#                decoded.append(d)
#            return decoded
#        else:
#            return data
#    else:
#        return -1


def parse_packet(receivedBytes):
    print("receivedBytes len =", len(receivedBytes))
    cabecera = receivedBytes[0]
    try:
        if cabecera == 0:
            return receivedBytes
        elif cabecera == 1:
            return receivedBytes
        elif cabecera == 2:
            return receivedBytes
        elif cabecera == 3:
            #print("Cabecera 3: Lidar Calidad ------------------")
            LidarCalidad = [receivedBytes[i] for i in range(1, 361)]
            return {'LidarCalidad': LidarCalidad}
        elif cabecera == 4:
            #print("Cabecera 4: Lidar Distancias ------------------")
            LidarDistancias = []
            Lidar90 = []
            Lidar270 = []
            for i in range(1, 721, 2):
                d = (receivedBytes[i] << 8) | receivedBytes[i + 1]
                LidarDistancias.append(d)
            for i in range(-5,5,1):
                Lidar90.append(LidarDistancias[90+i])
            for i in range(-5,5,1):
                Lidar270.append(LidarDistancias[270+i])
            return {'LidarDistancias': LidarDistancias,
                    'Distacias90' : Lidar90,
                    'Disrancias270' : Lidar270}
        elif cabecera == 5:
            #print("Cabecera 5: Lidar Distancias Tiempo ------------------")
            LidarMillis = []
            Lidar90m = []
            Lidar270m = []
            for i in range(1, 721, 2):
                d = (receivedBytes[i] << 8) | receivedBytes[i + 1]
                LidarMillis.append(d)
            for i in range(-5,5,1):
                Lidar90m.append(LidarMillis[90+i])
            for i in range(-5,5,1):
                Lidar270m.append(LidarMillis[270+i])
            return {'LidarDistancias': LidarMillis,
                    'Distacias90' : Lidar90m,
                    'Disrancias270' : Lidar270m}
        elif cabecera == 6:
            print("Cabecera 6: Informacion ------------------....................")
            #print("Contenido", receivedBytes)
            #print("Human =", " ".join(str(b) for b in receivedBytes))
            # Parse fields
            
            i = 1
            Millis = (receivedBytes[i] << 24 |
                        receivedBytes[i + 1] << 16 |
                        receivedBytes[i + 2] << 8 |
                        receivedBytes[i + 3])
            i = 5
            PosicionX = (receivedBytes[i] << 24 |
                        receivedBytes[i + 1] << 16 |
                        receivedBytes[i + 2] << 8 |
                        receivedBytes[i + 3])
            #print("posicionx"+str(PosicionX))
            i = 9
            PosicionY = (receivedBytes[i] << 24 |
                        receivedBytes[i + 1] << 16 |
                        receivedBytes[i + 2] << 8 |
                        receivedBytes[i + 3])
            i = 13
            PosicionXObjetivo = (receivedBytes[i] << 24 |
                                receivedBytes[i + 1] << 16 |
                                receivedBytes[i + 2] << 8 |
                                receivedBytes[i + 3])
            i = 17
            PosicionYObjetivo = (receivedBytes[i] << 24 |
                                receivedBytes[i + 1] << 16 |
                                receivedBytes[i + 2] << 8 |
                                receivedBytes[i + 3])
            i = 21
            Encoder = (receivedBytes[i] << 24 |
                    receivedBytes[i + 1] << 16 |
                    receivedBytes[i + 2] << 8 |
                    receivedBytes[i + 3])
            i = 25
            Estado = receivedBytes[i]
            i = 26
            Bateria = receivedBytes[i]
            i = 27
            firstBit = receivedBytes[i] >> 7
            angulo = (receivedBytes[i] << 24 |
                    receivedBytes[i + 1] << 16 |
                    receivedBytes[i + 2] << 8 |
                    receivedBytes[i + 3]) - ((firstBit) << 32)
            i = 31
            firstBit = receivedBytes[i] >> 7
            anguloObjetivo = (receivedBytes[i] << 24 |
                            receivedBytes[i + 1] << 16 |
                            receivedBytes[i + 2] << 8 |
                            receivedBytes[i + 3]) - ((firstBit) << 32)
            i =35
            tramo=receivedBytes[i]
            i = 36
            distancia0 = (receivedBytes[i] << 8) | receivedBytes[i+1]
            i = 38
            distancia90 = (receivedBytes[i] << 8) | receivedBytes[i+1]
            i = 40
            distancia270 = (receivedBytes[i] << 8) | receivedBytes[i+1]
            
            #print("Si,si.................")
            
            return {
                'Millis': Millis,
                'PosicionX': PosicionX,
                'PosicionY': PosicionY,
                'PosicionXObjetivo': PosicionXObjetivo,
                'PosicionYObjetivo': PosicionYObjetivo,
                'Encoder': Encoder-8192,
                'Estado': Estado,
                #'Bateria': Bateria,
               # 'Angulo': angulo,
               # 'AnguloObjetivo': anguloObjetivo,
                'Tramo': tramo,
               # 'Distancia0' : distancia0,
               #'Distancia90':distancia90,
                #'Distancia270':distancia270,
            }
        elif cabecera == 7:
            #print("Cabecera 7: Camera ------------------")
            i = 1
            firma1Detectada = receivedBytes[i] == 1
            i = 2
            firma1PosicionX = receivedBytes[i]
            i = 3
            firma1PosicionY = receivedBytes[i]
            i = 6
            firma2Detectada = receivedBytes[i] == 1
            i = 7
            firma2PosicionX = receivedBytes[i]
            i = 8
            firma2PosicionY = receivedBytes[i]
            #return {
             #   'firma1Detectada': firma1Detectada,
             #   'firma1PosicionX': firma1PosicionX,
             #   'firma1PosicionY': firma1PosicionY,
              #  'firma2Detectada': firma2Detectada,
             #   'firma2PosicionX': firma2PosicionX,
              #  'firma2PosicionY': firma2PosicionY,
           # }
        else:
            print("Cabecera Desconocida")
            return ("Cabecera desconocida...........")
    except Exception as e:
        print(e)

while True:
    data, addr = sock.recvfrom(1000000)
    #print("received")
    #decoded = decodeBytes(data)
    #print(decoded)
    print(parse_packet(data))
