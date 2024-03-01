# Climber Subsystem

The climber is used to ascend onto the chains of the STAGE at the end of the match. The climber mechanically consists of a single motor that behaves as a winch, retracting a telescoping arm. A constant force spring acts against the winch, pulling the climber into its extended position. Attached to the top of the arm assembly there's a hook which grabs onto the chain.

## Coordinate System

We use the following coordinate system for the climber:

- The origin (0) represents the climber while fully retracted.
- Extending the climber upwards increases the climber's position.
- Positive duty cycles sent to the climber motor cause the mechanism to extend upwards.

## Limits

It's important for the climber to stay within its normal operating conditions.

If the winch is overextended, the cable may lose tension and become tangled. If the climber is used in this state, it could likely cause a catastrophic failure.

Similarly, overtensioning the winch should be avoided. Excess tension is wasteful and risks damaging our climber motor.

We use a software limit (soft-limit) to prevent the climber from overextending. This code is outsourced to our motor controller. When the motor's internal encoder reaches a certain value, the motor controller will forbid additional extension.

Overtensioning is solved with a physical limit switch. When the climber is fully retracted, the limit switch is pressed, preventing additional tensioning. The limit switch also serves as an "auto home" functionality for the climber, providing a reference point for the soft-limit.

## Commands

### Auto Home

The climber is slowly retracted until the limit switch is pressed. If after a specified timeout the limit switch isn't pressed, then the climber's final position is considered zero, and a soft-limit is used for the rest of the match.

This command should be run before any other climber command to ensure the climber is in a known safe state.

### Extend

When run, the climber's winch is released, and the climber extends upwards. The climber will continue to extend until the soft-limit is reached or the command is interrupted.

### Climb Fast

The climber is retracted at full power, and power is cut when the limit switch is touched. This command is used to quickly retract the climber but might be "jittery" as the climber is constantly turned on and off.
