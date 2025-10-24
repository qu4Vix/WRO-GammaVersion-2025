# begin, time, position, speed, blocks, lidar
'''
class dataStruct:
    def __init__(self, keyWords) -> None:
        self.dataString = None
        self.keyWords = keyWords.replace(" ","").split(",")
        self.keyWords.remove("begin")
        self.dataParts = []
        print(self.keyWords)
    def split(self, string):
        #print(string)
        if type(string) != str:
            return
        self.dataString = string
        self.keyWords = ["begin"] + self.keyWords
        _stringChunks = string.replace(" ", "").split(",")
        _indices = []
        for keyWord in self.keyWords:
            for chunk in _stringChunks:
                if chunk == keyWord:
                    _indices.append(_stringChunks.index(chunk))
        #print(_indices)
        dataParts = []
        for keyWordIndex in _indices:
            set = []
            for index in range(_indices[_indices.index(keyWordIndex) - 1] + 1, _indices[_indices.index(keyWordIndex)]):
                set.append(_stringChunks[index])
            dataParts.append(set)
        # if the data string is wanted to not start with the begin:
        try:
            dataParts.pop(0)
        except:
            #print("not pop")
            pass
        self.keyWords.remove("begin")
        # end of begin omission
        self.dataParts = dataParts
        # output
        #print(self.keyWords)
        #print(dataParts)
'''
        
packetSizes = (5, 10, 15, 360, 720, 720, 39, 10)
TOTAL_PACKETS = len(packetSizes)
packetNames = {
    "Small" : 0,
    "Medium" : 1,
    "Big" : 2,
    "Lidar Quality" : 3,
    "Lidar Distances" : 4,
    "Lidar Millis" : 5,
    "General Information" : 6,
    "Camera Detecion" : 7
}

class dataPacket:
    def __init__(self, type):
        self.type = type
        self.size = packetSizes[type]
        self.data = list(0 for i in range(0, self.size))
    def setDataPacket(self, packet):
        if len(packet) == self.size + 1: # we include the data type
            self.data = packet
    def decode(self):
        try:
            if self.type == 0:
                return self.data
            elif self.type == 1:
                return self.data
            elif self.type == 2:
                return self.data
            elif self.type == 3:
                #print("Cabecera 3: Lidar Calidad ------------------")
                LidarCalidad = [self.data[i] for i in range(1, 361)]
                return {'LidarCalidad': LidarCalidad}
            elif self.type == 4:
                #print("Cabecera 4: Lidar Distancias ------------------")
                LidarDistancias = []
                Lidar90 = []
                Lidar270 = []
                for i in range(1, 721, 2):
                    d = (self.data[i] << 8) | self.data[i + 1]
                    LidarDistancias.append(d)
                for i in range(-5,5,1):
                    Lidar90.append(LidarDistancias[90+i])
                for i in range(-5,5,1):
                    Lidar270.append(LidarDistancias[270+i])
                return {'LidarDistances': LidarDistancias,
                        'Distacias90' : Lidar90,
                        'Disrancias270' : Lidar270}
            elif self.type == 5:
                #print("Cabecera 5: Lidar Distancias Tiempo ------------------")
                LidarMillis = []
                Lidar90m = []
                Lidar270m = []
                for i in range(1, 721, 2):
                    d = (self.data[i] << 8) | self.data[i + 1]
                    LidarMillis.append(d)
                for i in range(-5,5,1):
                    Lidar90m.append(LidarMillis[90+i])
                for i in range(-5,5,1):
                    Lidar270m.append(LidarMillis[270+i])
                return {'LidarMillis': LidarMillis,
                        'Distacias90' : Lidar90m,
                        'Disrancias270' : Lidar270m}
            elif self.type == 6:
                print("Cabecera 6: Informacion ------------------....................")
                # Parse fields
                
                i = 1
                Millis = (self.data[i] << 24 |
                            self.data[i + 1] << 16 |
                            self.data[i + 2] << 8 |
                            self.data[i + 3])
                i = 5
                PosicionX = (self.data[i] << 24 |
                            self.data[i + 1] << 16 |
                            self.data[i + 2] << 8 |
                            self.data[i + 3])
                i = 9
                PosicionY = (self.data[i] << 24 |
                            self.data[i + 1] << 16 |
                            self.data[i + 2] << 8 |
                            self.data[i + 3])
                i = 13
                PosicionXObjetivo = (self.data[i] << 24 |
                                    self.data[i + 1] << 16 |
                                    self.data[i + 2] << 8 |
                                    self.data[i + 3])
                i = 17
                PosicionYObjetivo = (self.data[i] << 24 |
                                    self.data[i + 1] << 16 |
                                    self.data[i + 2] << 8 |
                                    self.data[i + 3])
                i = 21
                Encoder = (self.data[i] << 24 |
                        self.data[i + 1] << 16 |
                        self.data[i + 2] << 8 |
                        self.data[i + 3])
                i = 25
                Estado = self.data[i]
                i = 26
                Bateria = self.data[i]
                i = 27
                firstBit = self.data[i] >> 7
                angulo = (self.data[i] << 24 |
                        self.data[i + 1] << 16 |
                        self.data[i + 2] << 8 |
                        self.data[i + 3]) - ((firstBit) << 32)
                i = 31
                firstBit = self.data[i] >> 7
                anguloObjetivo = (self.data[i] << 24 |
                                self.data[i + 1] << 16 |
                                self.data[i + 2] << 8 |
                                self.data[i + 3]) - ((firstBit) << 32)
                i = 35
                tramo=self.data[i]
                i = 36
                distancia90 = (self.data[i] << 8) | self.data[i+1]
                i = 38
                distancia270 = (self.data[i] << 8) | self.data[i+1]
                
                return {
                    'Millis': Millis,
                    'PosicionX': PosicionX,
                    'PosicionY': PosicionY,
                    'PosicionXObjetivo': PosicionXObjetivo,
                    'PosicionYObjetivo': PosicionYObjetivo,
                    'Encoder': Encoder,
                    'Estado': Estado,
                    'Bateria': Bateria,
                    'Angulo': angulo,
                    'AnguloObjetivo': anguloObjetivo,
                    'Tramo': tramo,
                    'Distancia90': distancia90,
                    'Distancia270': distancia270,
                }
            elif self.type == 7:
                #print("Cabecera 7: Camera ------------------")
                i = 1
                firma1Detectada = self.data[i] == 1
                i = 2
                firma1PosicionX = self.data[i]
                i = 3
                firma1PosicionY = self.data[i]
                i = 6
                firma2Detectada = self.data[i] == 1
                i = 7
                firma2PosicionX = self.data[i]
                i = 8
                firma2PosicionY = self.data[i]
                return {
                    'firma1Detectada': firma1Detectada,
                    'firma1PosicionX': firma1PosicionX,
                    'firma1PosicionY': firma1PosicionY,
                    'firma2Detectada': firma2Detectada,
                    'firma2PosicionX': firma2PosicionX,
                    'firma2PosicionY': firma2PosicionY,
                }
            else:
                print("Cabecera Desconocida")
                return False
        except Exception as e:
            print(e)