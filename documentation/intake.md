# Ground Intake

The ground intake collect NOTES off of the ground and feeds them into the [Indexer](indexer.md). 

The intake is fixed to the robot and is completely within the frame perimeter. It's made up of a series of compliant rollers that feed notes into the robot. The intake is powered by a single motor and we use an infrared (IR) sensor to detect when a note is in the intake.

The intake is driven with simple percent output control. The IR sensor is used to stop the intake when a note is detected. Stopping the intake in this competition helps excess damage to NOTEs.

## Coordinate System

Positive voltage brings notes into the robot. Negative voltage expel notes from the robot.

## Infrared Sensor

TODO: Write about the voltage divider and the specific sensor we use.
 
## Commands

### Run Slow

The Intake doesn't always perfectly complete the handoff to the Indexer. This command is used to slowly run the Intake to help ensure the Indexer gets the note during launches.

### Run Until Captured

The Intake runs until a note is detected by the IR sensor, plus a small delay to ensure the note is fully captured. This command also ties into the [LEDSubsystem](leds.md#roborio-java-usage) to indicate when a note is captured. 

Most "drive" commands are designed to run-forever while the driver holds the button. However, this command is an exception. It runs while the button is held and stops when either the button is released or a note is detected.

This command works great in Autonomous mode. We use it end "pickup paths" early as soon as a note is detected.
