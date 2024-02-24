# PH2024-alpha
Code for Team 418's Alpha Bot, a modified version of the Everybot. The robot's code is written in Java and is based off of WPILib's Java command based control system. It also makes use of our custom library [PurpleLib](https://github.com/lasarobotics/PurpleLib).

The code is organised into several subsystems, each responsible for a different aspect of the robot's functionality. This document explains this code.

## [Subsystems](src/main/java/frc/robot/subsystems)
### [Drive Subsystem](src/main/java/frc/robot/subsystems/drive/DriveSubsystem.java)
The drive subsystem controls the drivetrain of the robot, which is a 4 wheel differential drive, using Sparks and NEOs.

### [Shooter Subsystem](src/main/java/frc/robot/subsystems/drive/ShooterSubsystem.java)
The shooter subsystem controls the manipulation of game pieces and is composed of two Sparks and NEOs for intaking, indexing, and shooting pieces into the shooter.

### [Climber Subsystem](src/main/java/frc/robot/subsystems/drive/ClimberSubsystem.java)
The climber subsystem controls a hook that is raised and lowered using a SparkMax and NEO.

### [Amp Subsystem](src/main/java/frc/robot/subsystems/drive/AmpSubsystem.java)
The amp subsystem is a single SparkMax and a Neo 550 that uses a wheel to grip and drop a note into the amp.
