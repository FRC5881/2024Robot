# Shoother Subsystem

the shooter subsystem serves to launch an object at a specific speed, a specific distance. As well as this, itcan act as a source intake, to intake from a human player.

## constructer

The constructor, which has the same name as the IntakeSubsystem, just in camelCase; uses the CANSparkMax function to add sparkMAX number to the intakeMotor used in it. As well as this it also adds limits to motors, so they will not shoot the ring at a dangerous speed.

## methods

the methods used in this function are desgined to be multi-directional.

### periodic

this method is desgined to display the shooter speed on the smart dashboard, using the .putNumber command *this method is used for testing*.

### setSpeed

This method is desgined to set the speed of the motor at a speed based on the percentSpeed. It does this by telling the intakeMotor to spin at a speed set as a constant. This is done through the use of the .set function, which sets the speed to the constant set in Constants.java.

### isAtSetPoint

this method is desgined to tell the shooter when the object is at the velocity setpoint.

### stop

this method is desgined to stop the intakeMotor, this is done through the use of the .set function, which sets the speed to 0.

## commands

### cSetSpeed

this command is desgined to set the speed of the motors at the speed setSpeed is set to

### cShootHigh

this command is desgined to shoot an objet at kShooterHighSpeed

### cShootLow

this command is desgined to shoot an objet at kShooterLowSpeed

### cWaitForSetPoint

this command is desgined to make the shooter wait to shoot an object until it is at the right speed

### cSourceIntake

this command is desgined to intake through the shooter
