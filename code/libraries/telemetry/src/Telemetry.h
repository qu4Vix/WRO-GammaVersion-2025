/*
 * Telemetry.h - Library to send telemetry packets trough serial to a microcontroller which will send them trough Udp to a computer
 *
 * Created by the Gamma Version Team, 2025
 * 
 * 
 */

#ifndef TELEMETRY_h
#define TELEMETRY_h

#include <Arduino.h>

HardwareSerial teleSerial(0);

// send a piece of data to the telemetry esp32
void sendPacket(byte packetType, byte* packet);

void begin(byte RX, byte TX) {
    teleSerial.begin(1000000, SERIAL_8N1, RX, TX);
}

const uint16_t DataLength[8] = {5,10,15,360,720,720,41,10};

/* TELEMETRY STRING FORMAT
 *  |inicioTX               |PacketType|Data|
 *  0xAA,0xAA,0xAA,0xAA,    0x--,       0x--...0x--
 */
/*Dependiendo del tipo de paquete, éste contendrá cierta cantidad de datos
 *          -0 -> 5 bytes       -> NA
 *          -1 -> 10 bytes      -> NA
 *          -2 -> 15 bytes      -> NA
 *          -3 -> 360 bytes     -> Lidar Quality
 *          -4 -> 720 bytes     -> Lidar Distances
 *          -5 -> 720 bytes     -> Lidar millis
 *          -6 -> 39 bytes      -> General information
 *          -7 -> 10 bytes      -> Camera detection
 */
/* ENVIAMOS PAQUETE TIPO 3 CALIDAD MEDIDA */
/* ENVIAMOS PAQUETE TIPO 4 DISTANCIAS */
/* ENVIAMOS PAQUETE TIPO 5 INFORMACION GENERAL
 *
 *      --Millis (unsigned long) 4 bytes
 *      --Posicion x (long) 4 bytes
 *      --Posición y (long) 4 bytes
 *      --Posición x (long) Objetivo 4 bytes
 *      --Posición y (long) Objetivo 4 bytes
 *      --Encoder (uint32) 4 bytes
 *      --Estado (uint8) 1 byte
 *      --batería (uint8) 1 byte
 *      --Ángulo (long) 4 bytes
 *      --Angulo Objetivo (long) 4 bytes
 *      --Tramo (unit8) 1 byte
 *      --Distancia90 (unit16) 2 bytes
 *      --Distancia270 (unit16) 2 bytes
 * 
 *      --Cámara firma1 Detectada 1 byte
 *      --Cámara firma1 x 1 byte
 *      --Cámara firma1 y 1 byte
 *      --Camara firma1 width 1 byte
 *      --Camara firma1 height 1 byte
 *      --Cámara firma2 Detectada 1 byte
 *      --Cámara firma2 x 1 byte
 *      --Cámara firma2 y 1 byte
 *      --Camara firma2 width 1 byte
 *      --Camara firma2 height 1 byte
 *       
 *      |MMMM|XXXX|YYYY|MMMM|NNNN|QQQQ|E|B|AAAA|OOOO|T|RR|LL|
 *       0000 0000 1111 1111 2222 2222 3 3 3333 3344 4 44 44
 *       0000 1234 5678 9012 3456 7890 1 2 3456 7890 0 00 00
 * 
 *      |U|I|O|A|S|D|
 *       0 0 0 0 0 0
 *       1 2 3 4 5 6
 * 
 */

// Send the start of the packet header
void _sendheader() {
    for (int i = 0; i<4; i++) {
      teleSerial.write(0xAA);
    }
}

void _senddata(byte* pointer, int8_t size) {
  int8_t position = size - 1;   // We run trough the memory from greatest value to least, 
                                // since the receiver expects data in MSB order
  while (position >= 0 ) {
    teleSerial.write(pointer[position]);
    position--;
  }
}

void sendPacket(byte packetType, byte* packet) {
  byte size;
  if (sizeof(packet) == size);
  _sendheader();
  teleSerial.write(packetType);   //Send the packet type
  for (int i=0; i<size; i++) {    //Send the packet data
      teleSerial.write(packet[i]);
  }
}

struct dataPacket
{
  byte packetType;
  byte size;
};

class Packet {
    public:
    Packet(byte type);
    ~Packet();
    void addBuffer(byte datum);
    void addBuffer(uint16_t datum);
    void send();

    private:
    byte _size;
    byte _packetType;
    byte* _data;
    byte bufferPosition = 0;
};

Packet::Packet(byte Type) : _packetType(Type) {
    _size = DataLength[Type];
    _data = new byte[_size];
}

Packet::~Packet(){
    delete[] _data;
}

void Packet::addBuffer(byte datum) {
    _data[bufferPosition] = datum;
    bufferPosition++;
}

void Packet::addBuffer(uint16_t datum) {
    addBuffer(datum & 0xff);
    addBuffer(datum << 8);
}

void Packet::send() {
    _sendheader();                  // Send the header
    teleSerial.write(_packetType);  // Send the packet type
    _senddata(_data, _size);        // Send the packet data
}

#endif