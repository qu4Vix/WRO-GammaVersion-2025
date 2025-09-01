# WRO 2025 GammaVersion’s repository
**This is our repository for the 2025 season of the WRO Future Engineer Challenge**

***
<img src="https://github.com/qu4Vix/WRO-GammaVersion-2025/blob/main/readme-photos/Placeholder%20F.003.png?raw=true">

## Contents

## HARDWARE
Designing a self-driving car for these challenges requires a meticulous design process to achieve an optimal final result that meets the needs of the challenge and the potential obstacles that may arise during the programming process and the competition itself.

In our case, in order to reduce development times and finding it more feasible and simple to rely on existing resources than having to design a new chassis from scratch, we decided to use a remote-controlled car kit that we then modified extensively to adapt it to our particular needs.

### Car movement
We can highlight as the main modification the redesign of the turning system to allow tighter turns, since the original kit was deficient in this aspect and forced us to have to maneuver to be able to turn correctly during the competition and, although this is contemplated in the regulations, it made us lose a lot of time and increased the total complexity of the strategy. The redesign required the creation of new parts that have been designed by ourselves and whose 3D files can be found in our repository in the section [`robot-info`](/robot-info/).We have printed these parts with our 3D printer, which also reduces costs and allows us to easily create different versions if we see that they fail or not.
	
With these changes to the turning system we have implemented the **Ackerman's Steering Geometry**,this mechanism allows the turning wheels to take different paths when turning, since the turning radius of the inner wheel is smaller than that of the outer wheel, thus the angle of each turning wheel is different. This implementation allows for better turning, and with certain tweaks, we have minimized the required turning radius. This mechanism is actuated by a generic servo **DSM44 Servo**. It has the necessary torque and speed without being a problem in terms of power consumption. In the following images, we can see the nature of why this mechanism is necessary, a simplified version of how it works, and our design of the mechanism for our robot.

| ![ ](./readme-photos/Ackerman's%20Steering%20Geometry%20diagram.png) | ![ ](./readme-photos/Ackerman's%20Steering%20Geometry%20in%20our%20robot.png) |

More information about Ackerman's Steering Geometry can be found in [this Wikipedia article](https://en.wikipedia.org/wiki/Ackermann_steering_geometry).

According to competition regulations, only one motor can drive a drive axle. Therefore, in our case, we made the rear axle the drive axle to which the motor is connected. This is how the original kit was made. We also used the differential that came with the kit for proper operation. The differential allows the rear wheels to travel different distances, since the arc that each wheel travels is different when turning, as the turning radius of each wheel also varies. For our robot, we also used the motor that came with the kit; you can see the image below.

<img src = "https://github.com/qu4Vix/WRO-GammaVersion-2025/blob/main/readme-photos/other%20photos/Placeholder%20F.001.png?raw=true">

This motor provides us with the necessary torque to comfortably move the robot without it getting stuck. In our case, we're not trying to get the robot to go at maximum speed, so the robot's overall speed wasn't a relevant factor when choosing the motor.

### Structural design

As we can see in this image, the robot needs to accommodate a variety of different sensors in its limited space. This required a proper structural design beyond the kit's chassis, which is the lower black part of the robot.

First of all, we can distinguish the **LiDAR** Located on the front of the robot, this component rests on the chassis and is bolted to it. It also connects to a front bumper that protects the components in the event of a collision during testing or competition.

Secondly, below the LiDAR is the space needed to place the components of the **Ackerman's Steering Geometry** and the **servo** in charge of turning the wheels.

Thirdly, at the rear we can see from above the **PCB**, which houses much of our car's electronics, further on in [PCB](#PCB) we detail its design. Just below the PCB is the **motor** and the **differential** that drives the drive axle.

Finally, at the rear, the metal mast that raises and holds our **camera** is held on a 3D-designed base to the vehicle.

<img src = "https://github.com/qu4Vix/WRO-GammaVersion-2025/blob/main/readme-photos/other%20photos/Placeholder%20F.002.png?raw=true">

## ELECTRONICS AND SENSORS

Our robot is made up of a whole series of electronic components and sensors that work together to provide us with the best possible information so we can perform our tasks. We must emphasize the use of information from multiple sensors simultaneously to reduce errors.
	
Below we provide a detailed list of all the electronic components that make up the car, along with a detailed description of their functionality and the advantages that led us to use them.

### List of components
* DOIT ESP32 DevKit V1 x2
* HuskyLens	
* RPLiDAR A1 M8
* MPU 9250
* Pololu Magnetic Encoder
* DSM44 Servo
* Motor Driver x2
* Power Supply x3
* RGB LEDs

#### Esp-32 (Microcontroller)

To bring our autonomous car project to fruition, we opted for the ESP-32 microcontroller as the central control unit. The ESP-32 boasts several compelling advantages that played a pivotal role in our decision-making process:

1. Computing Power and Versatility: The ESP-32 distinguishes itself with its robust computational capabilities and remarkable versatility. Equipped with a potent dual-core processor and ample memory capacity, it emerged as the natural choice for our robotics endeavor, enabling rapid computation and accommodating our storage needs seamlessly.

2. Advanced Wireless Connectivity: The ESP-32 integrates cutting-edge Wi-Fi and Bluetooth connectivity, positioning it as a frontrunner for efficient wireless communication. This capability empowered us to establish seamless and reliable communication channels between the autonomous car and external devices, a really useful feature for our debugging needs. However, wireless connectivity is forbidden during the competition and we disable this option before the match.

3. Sensor and Actuator Compatibility: The ESP-32's adaptability extends to sensor and actuator integration. With its multitude of I/O (input/output) pins and versatile interface options, we effortlessly connected and controlled various sensors, including the IMU and LiDAR, as well as the steering servos.

<img src = "https://github.com/qu4Vix/WRO-GammaVersion-2025/blob/main/readme-photos/other%20photos/ESP32%20Pin%20out.jpg?raw=true">

We integrated two ESP32s into our robot, as it was necessary due to the large amount of information they must analyze from all the sensors and the decisions they must make. For this purpose, we call one ESP32 "Slave," to which several sensors are connected. After analyzing this information, it sends it to another ESP32, which we call "Master." Other sensors are connected to this ESP32, and it is responsible for executing all the movement commands.

<img src = "https://github.com/qu4Vix/WRO-GammaVersion-2025/blob/main/readme-photos/other%20photos/ESP32%20Diagram%20of%20connections.jpg?raw=true">

#### HuskyLens (Camera)

The HuskyLens camera was our choice because it gives us a larger horizontal field of view compared to other available cameras and also incorporates several algorithmic options to be able to detect a wide variety of things, such as distinguishing colors, people, objects, codes, etc. We also chose it because of its improved data processing capacity, which is superior to the alternatives we had locally and which also allows us to train it much better than other cameras, thus guaranteeing a great improvement in precision and allowing us to adjust to the lighting conditions of the competition venue.
	
The camera's sole function is to detect the colored blocks, indicate their position on the camera, and indicate their color. We train it for all of this before the competition.

<img src = "https://github.com/qu4Vix/WRO-GammaVersion-2025/blob/main/readme-photos/other%20photos/HuskyLens%20camera.jpg?raw=true">

#### RPLiDAR A1 M8 (LiDAR)

Rather than relying on traditional ultrasonic sensors for obstacle detection, we opted for LiDAR technology. LiDAR, which stands for Light Detection and Ranging, uses laser beams to measure distances to objects with exceptional precision. This technology offers several advantages over ultrasonic sensors:

1. Greater Range: LiDAR can detect objects at much greater distances, providing the car with more time to react to potential obstacles.

2. High Precision: LiDAR provides accurate distance measurements, resulting in better navigation and collision avoidance.

3. 360-Degree Coverage: LiDAR offers a full 360-degree view around the car, ensuring comprehensive situational awareness.

4. Environmental Adaptability: LiDAR is less affected by environmental factors such as ambient light, making it more reliable in various conditions.

During the match the LiDAR technology is used to establish the initial position of the robot and the direction of the game. In this way the random initial conditions do not affect the outcome of the match.

<img src = "https://github.com/qu4Vix/WRO-GammaVersion-2025/blob/main/readme-photos/other%20photos/RPLiDAR%20A1%20M8%20image.jpg?raw=true">

#### MPU 9250 and Pololu Magnetic Encoder (IMU and encoder)

On one hand, IMU stands for Inertial Measurement Unit, this is a combination of gyroscopes, accelerometers and in our MPU 9250 magnetometers too. On the other hand, a enconder is a system that reads the rotation of the motor or axle and allows us to know how much it has turned, and with some analysis it lets us know the length of the displacement of our robot. Inertial navigation has plenty of disadvantages compared to for example LiDAR navigation, some of its problems are that the precision is worse than LiDAR's and that this system does not really know where the robot is in the real world, only how much it has moved or turned since the start of the program. It might seem that using this technology is a downgrade and unnecessary as we already have the LiDAR, however, the IMU combinated with a encoder gives easier-to-use data compared to the huge amount of information the LiDAR gives us every second.

In our case, we only use our LiDAR to establish which is the initial position of the robot and the direction of the game, but after this the LiDAR is no longer used in the game. Through all the rest of the match only the IMU and encoder are used to follow an imaginary path around the central square in the Open Challenge or around the square and through the traffic signs in the Obstacle Challenge with the color information gived by the camera.

### Sensor list

As we can see, our robot only uses the following sensors, whose data are used in unison by our microcontrollers to maximize reliability and be able to solve the different challenges that this competition poses to us.
* LiDAR
* IMU
* Encoder
* Camera

### Power Management

Our robot is powered by a single 7V battery with a capacity of 1200 mAh, which provides the necessary energy for the robot to function properly. Before building the car, when we were considering which electronic components to choose, we considered the energy consumption of each component and how we could provide the necessary voltage to each element. After analyzing this, we designed the system to adequately meet the energy requirements.

Our PCB contains several power sources that distribute power correctly and efficiently to our sensors and actuators. We can list them as follows:

1. Dual 5-Volt Power Supplies: One 5-volt power supply was dedicated to powering the LiDAR system, while the other supplied power to the steering servos. This separation of power sources allowed us to optimize the energy delivery to these critical components, ensuring their reliable operation.

2. 3.3-Volt Power Supply: A dedicated 3.3-volt power supply was employed to energize the microcontrollers, IMU (Inertial Measurement Unit), motor drivers, and LED lights. This voltage level was carefully chosen to meet the requirements of these electronic components, guaranteeing stable and consistent performance.
	
By employing these distinct power sources, we effectively managed the power needs of our autonomous car's various subsystems, ensuring that each component received the appropriate voltage and current for reliable operation. This meticulous power management contributed to the overall success and stability of our robotics project.

### PCB

To seamlessly connect and control all the components of our autonomous car, we designed and printed a custom PCB. The PCB acted as the central hub that allowed us to interface with sensors, motors, microcontrollers, and other electronic components efficiently.

Our custom PCB design allowed for cleaner wiring, reduced interference, and enhanced reliability. It simplified the process of connecting and configuring various sensors and actuators, enabling smoother integration and troubleshooting during the development phase.

<img src = "https://github.com/qu4Vix/WRO-GammaVersion-2025/blob/main/readme-photos/other%20photos/PCB%20Design.jpg?raw=true">

<img src = "https://github.com/qu4Vix/WRO-GammaVersion-2025/blob/main/readme-photos/other%20photos/Schematic%20PCB.png?raw=true">

## STRATEGY AND OPERATION OF THE CODE

As we mentioned before, the code is divided into two parts, one for the ESP32 “Master” and another for the ESP32 “Slave”. In our case, the code for the “Slave” is the same for both types of rounds in the competition, the Open Challenge and the Obstacle Challenge. If you wish, you can activate or deactivate certain functions related to the Obstacle Challenge when not in use by changing a value of a define in the code, although this is not necessary for correct operation.

On the other hand, due to the large variations between the two challenges, we found two separate codes for the ESP32 "Master," one used only in the Open Challenge rounds and the other for the Obstacle Challenge rounds. This gives us two separate codes that allow us to better address the bugs and challenges of each category.

### How the “Slave” code works

The “Slave” code is responsible for collecting data from various sensors and sending it via serial to the “Master” for processing and analysis. On the other hand, the “Slave” receives data via serial to move the motor and servo from the “Master”.
	
The "Slave" also has the function of cleaning and filtering the camera information. Due to the way the HuskyLens works, it is very likely that it will mistakenly detect the orange line on the dashboard as if it were a red block, or even detect the parking barriers as red blocks. This is clearly detrimental to the robot's operation and can lead to errors. To avoid these errors, the "Slave" only sends to the "Master" the block that is closest to the camera and that is not a square or a horizontal rectangle (this is because the camera detects objects in squares that encompass the surface of the detected object, not the object's actual shape). This prevents detection errors.

### Location of the robot on the board

To determine the car's exact location on the dashboard, the same system is used in both the Open Challenge and the Obstacle Challenge. The system is based on a coordinate system that places (0,0) in the lower left corner from the car's point of view. To locate the car on the dashboard, two phases occur:

1. From the robot's starting location, the LiDAR measures the distance in a straight line from the robot to the front wall, that is, at an angle of 0° relative to the robot's current direction. With this data, we can determine the robot's current Y-position.

2. The robot then moves forward until it is a certain distance from the front wall that allows it to see to the sides. The robot now measures the distance to its left (270°) and right (90°). With this data, the robot is able to establish its position on the X axis of the coordinate system, and it also allows it to determine the direction of rotation of the circle.

### Strategy's Open Challenge

In the Open Challenge, the robot must complete three laps of the circuit from a random starting position and direction and must be able to finish in the same starting section. In addition, the size of the central square varies from round to round, adding additional difficulty.
	
With the information from the previous paragraph in mind, our strategy is based on correctly determining our position on the board and then completing the circuit from there. Our robot has no knowledge of the size of the central square, as it simply navigates an imaginary circuit that is larger than the maximum limits of the central square, as we can see in the images.

| ![ ](./readme-photos/Diagram%201%20Open%20Challenge.png) | ![ ](./readme-photos/Diagram%202%20Open%20Challenge.png) |

### Obstacle Challenge Strategy

In the Obstacle Challenge, the robot must complete three laps of the circuit, dodging the colored blocks so that the red block passes on the right and the green block on the left. Additionally, after completing the laps, the robot can park in the parking area marked with purple barriers. It can also start in this area for bonus points.
	
In our car, the straight sections of the board are organized into imaginary lanes, so that when the robot detects a colored block, it changes lanes or stays in the one it's currently in to pass on the correct side.

Due to the limited vision of our camera, the robot must perform corrective maneuvers during certain turns to view the blocks from the best angle for detection. Then, to park, we use several sensors installed on the sides of the robot.

## PHOTOS

### Car images

**PLACEHOLDER**

### Team images

**PLACEHOLDER**

## Demonstration videos

Demonstration videos of the car successfully completing both challenges. The links to each of them can be found in:

* [Open Challenge [PLACHEHOLDER]](https://www.youtube.com/watch?v=dQw4w9WgXcQ)
* [Obstacle Challenge [PLACEHOLDER]](https://www.youtube.com/watch?v=dQw4w9WgXcQ)

## License

The [source code](/code/) in this repository is licensed under the **GNU General Public License v3.0**.

The [hardware](/robot-info/hardware/) exposed in this project is licensed under the **Creative Commons Attribution Share Alike 4.0 International** license.

The documentation of this repository; found in [`robot-info`](/robot-info/), [`vehicle-photos`](/vehicle-photos/), [`videos`](/videos/), [`team-photos`](/team-photos/), as well as this `README.md`; is licensed under the **CERN Open Hardware Licence Version 2 - Strongly Reciprocal** license.

A copy of each license can be found in the [LICENSE](LICENSE) file. More information about the licenses in the specific README.md of each section.

## Credits

**PLACEHOLDER**

