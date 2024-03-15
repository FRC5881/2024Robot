# LED Subsystem

## Hardware Protocol

Our LED control system utilizes a custom one-way communication protocol between the FRC robot's RoboRIO and an Arduino-based LED controller. This protocol operates through an `AnalogOutput` port on our NavX-MXP board. The communication process is outlined as follows:

### Pattern Enumeration

Both the RoboRIO and the Arduino maintain a shared, hard-coded list of patterns, referred to as the [Patterns](#patterns).

Each pattern in this list is assigned a unique integer ID ranging from 0 to `NUM_PATTERNS`, inclusive.

### Pattern Selection

1. **Desired Voltage Calculation:**
   - To select a pattern, the RoboRIO computes a desired voltage corresponding to the chosen pattern ID.

2. **Voltage Transmission:**
   - The RoboRIO writes the computed desired voltage to the `AnalogOutput` port.

3. **Pattern Determination:**
   - The Arduino reads the voltage value received from the `AnalogOutput` port.
   - Based on this voltage, the Arduino selects the pattern with the closest matching voltage.

### Desired Voltage Calculation

The desired voltage is calculated using the following formula:

```c
float calculated_voltage(int id) {
    return 5.0 * ((float)id + 0.5) / NUM_PATTERNS;
}
```

## RoboRIO (Java) usage

### `PenningtonLEDs`

The `PenningtonLEDs` class implements the [Hardware Protocol](#hardware-protocol), it is fairly low-level and should not be used directly. Instead, we use the `LEDSubsystem` class, which provides a higher-level interface to the LED control system.

This class has an enum `RawPattern` that contains the "shared hard-coded list of patterns".

### `LEDSubsystem`

The `LEDSubsystem` class is a [Singleton](https://en.wikipedia.org/wiki/Singleton_pattern) that provides a higher-level interface to the LED control system. Commands call into this class to change the LED pattern.

If you read the list of `RawPatterns` you will see that many preform the same _motion_, but with different colors (ie `RED_CHASING`, `GREEN_CHASING`, `BLUE_CHASING`). `LEDSubsystem` abstracts over this by providing a smaller `Pattern` enum with a variant for each unique _motion_. Internally, `LEDSubsystem` will select the correct color based on the current Alliance color.

### Using Patterns

We use patterns by defining their lifetime in code. Currently, we use 2 lifetimes: "Robot Mode" and "Command".

By "Robot Mode" we're referring to long-lived patterns that may be active for minutes at a time. These patterns are typically set the `Robot.java` class. Patterns with this lifetime refer to Driver Station / FMS state of the robot.

- `DISABLED`
- `AUTONOMOUS`
- `TELEOP`
- `TEST`
- `E-STOP` (emergency stop)

By "Command" we're referring to short-lived patterns that are active for the duration of a `Command`. These patterns are tied to the lifecycle of a `Command`, typically being set in `initialize()` and reset in `end()`. Patterns with this lifetime refer to the state of the robot during a specific action.

"Command" patterns have higher precedence than "Robot Mode" patterns. If a "Command" pattern is set, it will override the "Robot Mode" pattern until the "Command" ends.

In `LEDSubsystem` we have 3 methods for setting these patterns:

```java
// Set a "Robot Mode" pattern
public void setDefault(Pattern pattern);

// Start/End a "Command" pattern
public void setOverride(Pattern pattern);
public void endOverride(Pattern pattern);
```

### Example Usage

```java
// Robot.java
public void autonomousInit() {
    // Set the default pattern for the autonomous "Robot Mode"
    LEDSubsystem.getInstance().setDefault(Pattern.AUTONOMOUS);
}

// SomeCommand.java
public void initialize() {
    // Override the "Robot Mode" pattern with a "Command" pattern
    LEDSubsystem.getInstance().setOverride(Pattern.RED_CHASING);
}

public void end(boolean interrupted) {
    // Be sure to end an override so that the "Robot Mode" pattern can be restored
    LEDSubsystem.getInstance().endOverride(Pattern.RED_CHASING);
}
```

## Arduino (C) usage

The Arduino should run an infinite loop with this structure:

```c
void loop() {
    // Read the voltage from the AnalogOutput port
    int voltage = analogRead(A0) * 5.0 / 1023.0;

    switch (determine_pattern(voltage)) {
        case PAT_NONE:
            // Set the LEDs to the "NONE" pattern
            pattern_solid(0, 0, 0);
            break;
        case PAT_RED_CHASING:
            // Set the LEDs to the "RED_CHASING" pattern
            pattern_chasing(255, 0, 0);
            break;
        // ...
    }
}
```

The `determine_pattern` function should return the pattern ID that is closest to the given voltage. This function should be implemented in a way that is consistent with the `calculated_voltage` function in `PenningtonLEDs`.

Patterns should be enumerated as a list of `#define` statements, with each pattern ID being assigned a unique integer value.

Pattern implementations can include any necessary logic to control the LEDs, including the use of loops, delays, and other control structures. Each call to a pattern implementation is expected to be blocking and is called a "cycle". Cycles should be as
short as possible to produce the desired visual effect. For example, `pattern_solid` has an extremely short cycle-time, only needing to set the color of the LEDs once, while `pattern_chasing` has a longer cycle-time, needing to shift the color of the LEDs across a strip.

Finally, the Arduino project should be included in our main [2024Robot](https://github.com/FRC5881/2024Robot) repository.

## Patterns

| ID | Name               |
|----|--------------------|
| 0  | SLOW_RAINBOW       |
| 1  | SOLID_RED          |
| 2  | SOLID_GREEN        |
| 3  | SOLID_BLUE         |
| 4  | BREATHING_RED      |
| 5  | BREATHING_GREEN    |
| 6  | BREATHING_BLUE     |
| 7  | POWER_DOWN         |
| 8  | SLOW_FLASH_GREEN   |
| 9  | CHASING_UP_RED     |
| 10 | CHASING_UP_GREEN   |
| 11 | CHASING_UP_BLUE    |
| 12 | CHASING_DOWN_RED   |
| 13 | CHASING_DOWN_GREEN |
| 14 | CHASING_DOWN_BLUE  |
| 15 | FAST_FLASH_RED     |
| 16 | FAST_FLASH_GREEN   |
| 17 | FAST_FLASH_BLUE    |
| 18 | SOLID_PURPLE       |
| 19 | FAST_RAINBOW_FLASH |
