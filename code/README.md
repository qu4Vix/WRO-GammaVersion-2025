# Code
This folder contains the code for our car.
The code is structured in Platform IO projects. Each microcontroller has its own project, each of which should be uploaded to the corresdponding micro.
## Platform IO
Platform IO is an extension for Visual Studio Code, which enables uploading arduino framework code to a compatible microcontroller, in this case am Espressif ESP32 Devkit V1.
## Projects
* ESP32 Master. Controls the main algorithm, the lidar, the mpu and the telemetry. It communicates with the ESP32 Slave through the serial 1 and with the ESP32 Telemetry through serial 0.
* ESP32 Slave. Controls most of the hardware: the motors, the servo, the camera, the encoder and the battery. It communicates through serial 1 with the ESP32 Master.
* ESP32 Telemetry. Receives data from the ESP32 Master and sends it through WiFi to a computer. This part is for feedback purposes only, hence it is disabled during official rounds by removing the Beetle unit, to comply with the rules of the competition.