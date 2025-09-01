# WRO 2025 GammaVersion’s repository
**This is our repository for the 2025 season of the WRO Future Engineer Challenge**

***
<img src="https://github.com/qu4Vix/WRO-GammaVersion-2023/blob/main/team-photos/official-photo.jpeg?raw=true">

## Contents

## HARDWARE
Designing a self-driving car for these challenges requires a meticulous design process to achieve an optimal final result that meets the needs of the challenge and the potential obstacles that may arise during the programming process and the competition itself.

In our case, in order to reduce development times and finding it more feasible and simple to rely on existing resources than having to design a new chassis from scratch, we decided to use a remote-controlled car kit that we then modified extensively to adapt it to our particular needs.

### Car movement
We can **highlight** as the main modification the redesign of the turning system to allow tighter turns, since the original kit was deficient in this aspect and forced us to have to maneuver to be able to turn correctly during the competition and, although this is contemplated in the regulations, it made us lose a lot of time and increased the total complexity of the strategy. The redesign required the creation of new parts that have been designed by ourselves and whose 3D files can be found in our repository in the section[3D MODELS, movement]We have printed these parts with our 3D printer, which also reduces costs and allows us to easily create different versions if we see that they fail or not.
	
With these changes to the turning system we have implemented theAckerman's Steering GeometryThis mechanism allows the turning wheels to take different paths when turning, since the turning radius of the inner wheel is smaller than that of the outer wheel, thus the angle of each turning wheel is different. This implementation allows for better turning, and with certain tweaks, we have minimized the required turning radius. This mechanism is actuated by a generic servo.DSM44 ServoIt has the necessary torque and speed without being a problem in terms of power consumption. In the following images, we can see the nature of why this mechanism is necessary, a simplified version of how it works, and our design of the mechanism for our robot.
