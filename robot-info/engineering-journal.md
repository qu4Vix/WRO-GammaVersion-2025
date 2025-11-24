# Engineering journal
In this journal we expose the process of the engineering of our robot.
The project starts in 2023, the last time our team competed in the WRO. From that car we have set the base for our current version.

# 2025

## June 26th - Initial commit and team constitution
First meeting of the team. We dismantled the car for cleaning and checking the state of the pieces.
The repository was created and the first commit was done. The README was created and the LICENSE was uploaded.

## July 1st - Repo structured and car rebuilt
After cleaning the car, we placed every component as in 2023 except for the camera.
In 2023 we found several issues with the Pixy2 camera during the National Final, bringing our run to an end.
The brightness of the camera could not be brought down from the maximum, hence disabling the car from distiniguishing any color from white.
This year, we have replaced the Pixy 2 for a HUSKY Lens, hoping for a better performance.
The HUSKY Lens, as well as the Pixy, has colour detection built-in to the camera. This allows an easier handling of the recognition task, since we only use the camera to detect the colour of the traffic lights.
We created folders to organise the GitHub repository, and uploaded a pinout for the new camera.
Finally, we designed and uploaded a screw spacer for the PCB, in order to prevent the PCB from being damaged by the screws.

## July 2nd - Update of the camera pinout
After a bit of research, we found out that the ESP32 microcontroller can take any two GPIOs as I2C ports.
This enables us to reuse the connections from 2023. The Pixy 2 supports SPI, contrarily to the HUSKY, but we may use the previous pins and set them up as I2C trough software.
The updated pinout was uploaded.

## July 3rd - ESP32 Master and Slave created
We created two Platform IO projects, one with the code for the ESP32 Master and another for the Slave.
We added code from the 2023 repo, which we will further correct and develop.

## July 7th - Phase 1 started
Today we started working on the Phase 1 of the challenge
We setup the Husky in the Slave to avoid the port being missused.
The telemetry was shortened in the Phase 1 so that we don't have to send the blocks detected, as it is not needed.
The README file of the code was drafted.

## July 10th - Reset problem handling
Today we encountered the first big problem. The car suddenly reboots during the laps. In 2023 this scarcely happened, but now it occurs almost every time we run the car.
In order to debug, we sent the error message of both ESP32 Master's cores via Telemetry.
On a first attempt to solve this issue we update a watchdog timer at the beggining the Lidar Task.
Attempt failed, the car kept rebooting with the same error messages.

## July 15th - Lidar work
On another, unsuccessful attempt of solving the reset issue, we disabled the watchdog timer of the core dedicated to the LIDAR.
The Lidar seems to take too long measuring distances to decide the turn sense, so we modify the readDistance function, which still doesn't work as wanted.
For debugging purposes, we store the distances to the right and to the left of the robot, to send them trough telemetry.

## July 17th - Simple camera code and corrections to the master
Simple code added to the ESP32 Slave to measure colour blocks with the HUSKY and send them to the Master.
We noticed a deviation of the car on start, due to the servo's 0 angle not being properly adjusted. We update the servo MIN and MAX in our CServo library to deal with it
We sent the reset reason of the Slave's cores to the master, whcih then sends it rough telemetry, trying to find out the reason of the rebooting of the car.
For quicker launches when testing the car, we created the PRACTICE_MODE define, which starts the car without expenting a button press when on true.
To check the conversion between encoder updates and milimeters, we created a test state "Prueba", which moves the car 3 "car" meters to calculate the quotient.
We further worked on the decideTurn and the readDistance functions, since they were still run to slow.

## July 19th - Phase 1 running
ReadDistance fucntion was finally working, after accounting for various measures and fixing a bug where the index of the array was set negative.
The reset error was finally explained and solved. There was a voltage cutoff due to the 3.3v power supply having too much current demand. It was solved by putting the ESP32 and the camera in different circuits. As a future task it would be better to design a new PCB.
The code folder was divided in two sub-folders, official- for the code that we wil use during the competition-, and extras.

## July 20th - Telemetry creation
ESP32 Telemtry was created to upload to the ESP32 Beetle which handles the telemetry. It receives bytes from the teleSerial port and sends them trough udp packets to a computer, specified by its IP.
Telemetry receivers were created to receive, decode and process the telemetry data sent by the ESP32 Master trough different udp packets.

## July 25th - Obstacle challenge code
The ESP32 Master Obstacles project was created to handle the obstacle challenge. It uses the same algorithm as the ESP Master, using switches for the checkTurn and Turn functions. The colour blocks are detected and change the objective position of each lane.
The code was imported from the 2023 repository.

## July 29th - Final Open Challenge corrections
The distances array from the lidar was duplicated before reading the distances so that writing and reading don't clash, since they occur in different cores. Also older measurements were accepted. Now decideTurn gets the turn sense quicker yet reliably, enabling us quicker completion times.

## July 30th - Camera tests
We created a Camera Testing project to test the code for the camera and colour block handling separetly from the main program.
The tested code was added to the ESP32 Slave, which will operate the camera in the main program.
From all the blocks detected, the heighest block is passed to the ESP Master, which will use this information to move the car.

## August 18th & 21st Documentation update
The repo README was sketched and the licenses were uploaded to the main license file for easier access.
We uploaded the STL and FreeCAD of the 3D models for the 3D printer, the parts being: steering and support components, wheels, sensor supports and PCB spacers. Included a README describing the folder structure and purpose of the models.

## August 22nd Update after holidays
Today's changes were mostly to get everything prepared before continuing to make changes. Changes were put up to date and some things were reorganised.
Telemetry was moved from official to extras for organisation purposes. The copy of ESP32 Telemetry, Telemetria 2.1 was removed since its changes were not relevant.
The code for the ESP32 Masters was syncronised. The updated LIDAR code of the Open Challenge was added to the Obstacle Challenge. The 2nd rounds speed system - based on defines for easier configuration - was implemented in the 1st round.
Innecesary states were removed; added track constants and the ENABLE_TELEMETRY define was added to the Open Challenge; practice mode was added to the phase 2 code.
Telemetry was unified, distance90 and distance270 were added to phase 2.
Finally, an problem with the transition from state e::Inicio to e::Recto, in which if the turnSense was decided too soon, e::Recto would never be reached.

## August 24th Camera block selection logic
We continued working on the Camera Testing project to work on the camera code. We aim for a better detection of the colour blocks by filtering the colour blocks depending on their width, height and the ratio of both. Since the blocks are rectangular prisms, we aim to detect an upward rectangle, such that the height/width ratio is greater than 1.
Some improvement to the telemetry has been made, spliting the camera data into another packet so that it only gets sent when there is new data. Additionally the age of the lidar measurements has been sent.

## August 26th Small Slave camera changes
Initialised the Husky to the Color Detection Algorithm by default, checked the conexion state and requested blocks before calling our filter function.
Added the decodification of the camera data to the receiver. Sent arrayBlocks through telemetry for an easier check of the colour blocks detected. This way we could check the filter algorithm better. Finally, for an easier debug, we have sent the millis as a packet timestamp.
We have added a header to each source file of our official projects, which refers to our repository and indicates the license associated with the file.

## September 1st README updates
We have further written the README file to better document our project. Images were added for an easier explanation of the robot.

## September 2nd Exiting the parking
Now that we have sorted the colour block detection we deal with the new feature introduced this year, parking.
We created a new project in extras to tackle the parking slot start and parking. We first intend to solve the start from the slot to test our car's steering on the slot.
Starting on the parking slot, we have decided that our robot should measure the distance to both side walls, whichever is greater shows the side to which we should turn to exit the slot. It turns out that we can use this information to decide the sense of turn and to set the X positon from start. On the other hand, now we cannot measure the Y distance from start since this distance is blocked by the slot walls.
Since the parking slot prevents traffic lights to be placed on the exterior of the 1st lane, we have moved inwards the exterior path of the car on the lane, preventing collisions with the parking slot.
We have deactivated the function that made the car turn around on the final lap. We have also incremented the speed increment to the 6th turn, but this doesn't seem to work.