# Intake

This subsystem is reletively simple, consisting of two methods and one command, it's only job is to supply the indexer with objects.

## constructer

The constructor, which has the same name as the IntakeSubsystem, just in camelCase; uses the CANSparkMax function to add sparkMAX number to the intakeMotor used in it.

## methods

The methods for this function are suck, and stop. This subsystem contains not ejection, or reversal system, as the desgin does not deem it necessary.

### suck

This method is desgined to suck in a note, it does this by telling the intakeMotor to spin at a speed set as a constant. This is done through the use of the .set function, which sets the speed to the constant set in Constants.java

### stop

this method is desgined to stop the intakeMotor, this is done through the use of the .set function, which sets the speed to 0.

## commands

the only command utilized in this subsystem is the cRun command, this is because it only has the nececity of supplying the indexer.

### cRun

this command is desgined to make the robot intake an object. it does this by first executing the suck function, and then executing the stop function. This is done through the utilization of a runEnd function, which will first execute the suck, and then stop command.
