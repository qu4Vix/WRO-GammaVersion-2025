# Code
This folder contains the code for our car.
The code is structured in Platform IO projects. Each microcontroller has its own project, each of which should be uploaded to the corresdponding micro.
## Platform IO
Platform IO is an extension for Visual Studio Code, which enables uploading arduino framework code to a compatible microcontroller, in our case either of our Espressif ESP32 Devkit V1.
## Folders
* **Official** This folder contains the main code of our project, which is to be uploaded for the official rounds of the competition. It is divided in 3 projects, one for the "Slave" ESP32 and two -one for each challenge- for the "Master" ESP32.
* **Extras** Here we save the code for tests, extras, tuning programs and telemetry senders, receivers and managers -which are here since telemetry is not allowed during official rounds-.
* **Libraries** Directory intended for general, shared code. Pin assignment headers can be located here, which define each usable pin of the ESP32s in our car. The mpu code, which is shared between both official "Master" projects, is located here for sinchronous updates.
## Projects
* **ESP32 Master**. Controls the main algorithm, the lidar, the mpu and the telemetry. It communicates with the ESP32 Slave through the serial 1 and with the ESP32 Telemetry through serial 0.
* **ESP32 Slave**. Controls most of the hardware: the motors, the servo, the camera, the encoder and the battery. It communicates through serial 1 with the ESP32 Master.
* **ESP32 Telemetry**. Receives data from the ESP32 Master and sends it through WiFi to a computer. This part is for feedback purposes only, hence it is disabled during official rounds by removing the Beetle unit, to comply with the rules of the competition.