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

## July 17th - Simple camera code
Simple code added to the ESP32 Slave to measure colour blocks with the HUSKY and send them to the Master.
We noticed a deviation of the car on start, due to the servo's 0 angle not being properly adjusted. We update the servo MIN and MAX in our CServo library to deal with it
We sent the reset reason of the Slave's cores to the master, whcih then sends it rough telemetry, trying to find out the reason of the rebooting of the car.


## July 19th
