import socket

LOCAL_UDP_IP = "192.168.0.101"
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
        if cabecera == 3:
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
            print("Cabecera 5: Informacion ------------------....................")
            #print("Contenido", receivedBytes)
            #print("Human =", " ".join(str(b) for b in receivedBytes))
            # Parse fields
            i = 1
            
            PosicionX = (receivedBytes[i] << 24 |
                        receivedBytes[i + 1] << 16 |
                        receivedBytes[i + 2] << 8 |
                        receivedBytes[i + 3])
            print("posicionx"+str(PosicionX))
            i = 5
            PosicionY = (receivedBytes[i] << 24 |
                        receivedBytes[i + 1] << 16 |
                        receivedBytes[i + 2] << 8 |
                        receivedBytes[i + 3])
            i = 9
            PosicionXObjetivo = (receivedBytes[i] << 24 |
                                receivedBytes[i + 1] << 16 |
                                receivedBytes[i + 2] << 8 |
                                receivedBytes[i + 3])
            i = 13
            PosicionYObjetivo = (receivedBytes[i] << 24 |
                                receivedBytes[i + 1] << 16 |
                                receivedBytes[i + 2] << 8 |
                                receivedBytes[i + 3])
            i = 17
            Encoder = (receivedBytes[i] << 24 |
                    receivedBytes[i + 1] << 16 |
                    receivedBytes[i + 2] << 8 |
                    receivedBytes[i + 3])
            i = 21
            Estado = receivedBytes[i]
            i = 22
            Bateria = receivedBytes[i]
            i = 23
            angulo = (receivedBytes[i] << 24 |
                    receivedBytes[i + 1] << 16 |
                    receivedBytes[i + 2] << 8 |
                    receivedBytes[i + 3])
            i = 27
            anguloObjetivo = (receivedBytes[i] << 24 |
                            receivedBytes[i + 1] << 16 |
                            receivedBytes[i + 2] << 8 |
                            receivedBytes[i + 3])
            i =31
            tramo=receivedBytes[i]
            i = 32
            distancia90 = (receivedBytes[i] << 8) | receivedBytes[i+1]
            i = 34
            distancia270 = (receivedBytes[i] << 8) | receivedBytes[i+1]
            
            '''
            i = 31
            firma1Detectada = receivedBytes[i] == 1
            i = 32
            firma1PosicionX = (receivedBytes[i-1] << 8) | receivedBytes[i]
            i = 33
            firma1PosicionY = receivedBytes[i]
            i = 34
            firma2Detectada = receivedBytes[i] == 1
            i = 35
            firma2PosicionX = (receivedBytes[i-2] << 8) | receivedBytes[i-1]
            i = 36
            firma2PosicionY = receivedBytes[i]
            # arrayTramo
            arrayBloques = [0]*8
            indice = 7
            for j in range(37, 45):
                arrayBloques[indice] = receivedBytes[j]
                indice -= 1
            tramo = receivedBytes[45]
            '''
            print("Si,si.................")
            
            return {
                'PosicionX': PosicionX,
                'PosicionY': PosicionY,
                'PosicionXObjetivo': PosicionXObjetivo,
                'PosicionYObjetivo': PosicionYObjetivo,
                'Encoder': Encoder,
                'Estado': Estado,
                'Bateria': Bateria,
                'angulo': angulo,
                'anguloObjetivo': anguloObjetivo,
                'distancia90':distancia90,
                'distancia270':distancia270,
            }
            '''
                'firma1Detectada': firma1Detectada,
                'firma1PosicionX': firma1PosicionX,
                'firma1PosicionY': firma1PosicionY,
                'firma2Detectada': firma2Detectada,
                'firma2PosicionX': firma2PosicionX,
                'firma2PosicionY': firma2PosicionY,
                'arrayBloques': arrayBloques,
                'tramo': tramo
                '''
        else:
            print("Cabecera Desconocida")
            return ("Cabecera desconocida...........")
    except Exception as e:
        print(e)

while True:
    data, addr = sock.recvfrom(1000000)
    print("received")
    #decoded = decodeBytes(data)
    #print(decoded)
    print(parse_packet(data))
