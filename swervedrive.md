# Swerve Drive

For our competetion bot we're using a swerve drive with a typical 4 module layout. 

## Hardware used

We purchased our modules from Swerve Drive Specialties with the L1 gear ratio and 4" billet wheels.

Drive gear ratio is 8.14: 1
Aziumuth gear ratio is 12.8: 1

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

### M1C2

#### Measured CANCoder Offsets

| Module      | Offset (degrees) |
|-------------|------------------|
| Front Left  | 223.505859       |
| Front Right | 221.133          |
| Back Left   | 319.306641       |
| Back Right  | 252.510          |

#### Module Locations

M1C2 has a 29" x 29" chassis with wheels inset 3" on each side. This makes a 23" x 23" sqaure of module locations. 23"/2 = 11.5" which is 0.2921 meters

| Module      | (x, y meters)      |
|-------------|--------------------|
| Front Left  | (+0.2921, +0.2921) |
| Front Right | (+0.2921, -0.2921) |
| Back Left   | (-0.2921, +0.2921) |
| Back Right  | (-0.2921, -0.2921) |

### Competetion Bot

#### Measured CANCoder Offsets

| Module      | Offset (degrees) |
|-------------|------------------|
| Front Left  |                  |
| Front Right |                  |
| Back Left   |                  |
| Back Right  |                  |

#### Module Locations

Our competetion chassis is 27" x 27". Modules are 5 1/2" x 5 1/2" and the drive wheel should be centered. 27 - 11 = 16" sqaure of module locations. 8" is 0.2032 meters

| Module      | (x, y meters)      |
|-------------|--------------------|
| Front Left  | (+0.2032, +0.2032) |
| Front Right | (+0.2032, -0.2032) |
| Back Left   | (-0.2032, +0.2032) |
| Back Right  | (-0.2032, -0.2032) |
