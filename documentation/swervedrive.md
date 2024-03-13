# Swerve Drive

For our competition bot we're using a swerve drive with a typical 4 module layout. 

## Hardware used

We purchased our modules from Swerve Drive Specialties with the L1 gear ratio and 4" billet wheels.

Drive gear ratio is 8.14: 1
Azimuth gear ratio is 12.8: 1

All of our swerve motors are REV NEO motors.

CTRE CANCoders are used as absolute azimuth sensors.

We use a navX2 as a gyroscope.

## Software

We use BroncBotz Team 3481's [Yet Another Generic Swerve Library](https://github.com/BroncBotz3481/YAGSL) (YAGSL) as the core to our swerve drive subsystem. We like to use this library because it handles odometry well.

## Layout

### CAN IDs

In CAN devices must have unique (id, name) pairs. This means no two SparkMaxes can have the same id, and no two CANCoders can have the same id, but a CANCoder and a SparkMax may share the same id.

D: Drive Motor
A: Azimuth Motor
θ: Azimuth CANCoder

```
          FRONT  +x
   ╔══════╦═══════╦══════╗
   ║ D: 1 ║       ║ D: 2 ║
   ║ A: 5 ║       ║ A: 6 ║ R
 L ║ θ: 1 ║       ║ θ: 2 ║ I
 E ╠══════╬═══════╬══════╣ G
 F ║      ║       ║      ║ H
 T ║      ║ (0,0) ║      ║ T
   ║      ║       ║      ║ 
 + ╠══════╬═══════╬══════╣ -
 y ║ D: 3 ║       ║ D: 4 ║ y
   ║ A: 7 ║       ║ A: 8 ║ 
   ║ θ: 3 ║       ║ θ: 4 ║
   ╚══════╩═══════╩══════╝
          BACK   -x
```
