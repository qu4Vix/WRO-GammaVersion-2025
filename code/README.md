# Code

This folder contains the code for our car.
The code is structured in Platform IO projects. Each microcontroller has its own project, each of which should be uploaded to the corresdponding micro.

## Platform IO

Platform IO is an extension for Visual Studio Code, which enables uploading arduino framework code to a compatible microcontroller, in our case either of our Espressif ESP32 Devkit V1. Platform IO can be installed by opening Visual Studio's extensions tab, searching "PlatformIO IDE" in the Marketplace Search box, clicking on it and then on the blue "Install" button.

### Uploading the code

For uploading the code, open the appropiate project, depending on the ESP32 that you want to flash and the Challenge that you wish to face. Connect the ESP32 correspondingly via a mirco USB cable to USB, through which the binary will be sent. When you open a platform IO project, the extension detects the platformio.ini file int the folder and automatically activates itself. When PlatformIO is active, an Upload button apperas on Visual Studio's bottom toolbar, which has the appearence of an arrow pointing right. Click on it and a terminal will open indicating the upload process. PlatformIO should automatically detect the COM serial designated and use it to upload the code to the ESP32. In case of an incorrect COM serial being used, select it manually by using the toolbar button with a cable in it and choosing the correct COM. If the upload process errors with message ESP32 in wrong bootmode, it may be solved by pressing the ESP32's BOOT button on upload. If the error message is could not connect to ESP32, check the connection and the upload port.

## Folders

* **Official** This folder contains the main code of our project, which is to be uploaded for the official rounds of the competition. It is divided in 3 projects, one for the "Slave" ESP32 and two -one for each challenge- for the "Master" ESP32.
* **Extras** Here we save the code for tests, extras, tuning programs and telemetry senders, receivers and managers -which are here since telemetry is not allowed during official rounds-.
* **Libraries** Directory intended for general, shared code. Pin assignment headers can be located here, which define each usable pin of the ESP32s in our car. The mpu code, which is shared between both official "Master" projects, is located here for sinchronous updates.

## Projects

* **ESP32 Master**. Controls the main algorithm, the lidar, the mpu and the telemetry. It communicates with the ESP32 Slave through the serial 1 and with the ESP32 Telemetry through serial 0.
* **ESP32 Slave**. Controls most of the hardware: the motors, the servo, the camera, the encoder and the battery. It communicates through serial 1 with the ESP32 Master.
* **ESP32 Telemetry**. Receives data from the ESP32 Master and sends it through WiFi to a computer. This part is for feedback purposes only, hence it is disabled during official rounds by removing the Beetle unit, to comply with the rules of the competition.
