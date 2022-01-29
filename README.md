# FRC4403_2022 :ram:
FIRST Robotics Competition code :computer: for the 2022 season. :video_game: 

## Purpose :wrench:
The code in this project was created for [RobotName], robot of Team 4403 ROULT of PrepaTec Campus Laguna. {nombre de robot} was designed to participate in the FIRST Robotics Competition (FRC) 2022 season, Rapid React :copyright:.

## Content :book:
This project contains the code used to control different parts of [RobotName]. It contains subsystems wich organize functions of the robot into classes, commands which execute said functions and command groups which form sequential or parallel execution of commands. 

Different constants are also organaized into classes and subclasses for better readability. Finally, the WPILib Shuffleboard is used to display a variety of widgets containing robot information for drivers to make the best choices during a match.

## Auto :trophy:
[RobotName] uses a variety of sensor to control its own functions during autonomous play. It uses encoders, gyroscopes, color sensors, ultrasonic sensors and computer vision to exectute pre-programmed tasks for the first 15 seconds of a match. 

## Subsystems :clipboard:
[RobotName] has 5 different subsystems that control different mechanisms on the robot. These subsystems are:
| Subsystem | Function |
|--------|--------|
|Drivetrain|Controls [RobotName] movement and gear selection. Contains sensors used in autonomous.|
|Intake|Used to collect Cargo during a match, uses a motor and a piston to extend itself out of [RobotName]'s frame perimeter.|
|Index|Used manipulate Cargo once inside [RobotName], transports Cargo from the Intake to the Shooter.|
|Shooter|Used to shoot Cargo out of [RobotName], uses a turret to aim at the target and an adjustable hood to vary the Cargo's launch angle.|
|Climber|Used during the endagme period of a match, it hangs [RobotName] from the mid rung and transfers it from one rung to the next.|

## Vision :camera:
 [RobotName] uses computer vision to find and align itself with the shooting target. The Vision code was written in pyton using CV2 and executed in a coprocessor. Our coprocessor of choice is a Raspberry 4 running WPILib's vision image. 

## Subautonomous Routines :art:
 [RobotName] uses multiple command groups to abstract different robot functions, these are automated to the point the driver only needs to schedule one command for a major functionality to be succesfullly executed. For example, only a shoot command needs to be scheduled for the robot to aim, check for Cargo in the Index, adjust the hood, spin up the shooter and shoot when it is ready.

<br>
<div style="text-align: center;">
<img src="roult.jpg" height="200" width="200"> <br/>
Team ROULT 4403.  
</div>