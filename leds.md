# LED Color

This is the colors that that the LEDs will show certain based on the bot's behavior

## Protocol

Our LEDs are driven by an ardunio. The protocl is as follows:

1. On the RIO enable a digital output PIN triggering an interrupt
2. The ardunio interrupts into a function that calls [pulseIn](https://www.arduino.cc/reference/en/language/functions/advanced-io/pulsein/)
3. The RIO calls [pulse](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DigitalOutput.html#pulse(double))
    - Hopefully we don't need to add any delay between 2 and 3
4. The pattern changes, and control is restored to the infinite loop

## Pattern

### Alliance Colors (lowest prioity)

#### Red Alliance

This command sets the color of all LEDs to #FF0000, this is used for red alliance

#### Blue Alliance

This command sets the color of all LEDs to #0000FF, this is used for blue alliance

#### No Alliance

This command sets the color of all LEDs to #00FF00, this is used for testing, and showing

### Period Pattern (medium priority)

#### Auto

This command activates LEDs in a breathing strobe pattern and sets LEDs to color based on alliance

#### Teleop

This command activates LEDs in a solid pattern and sets LEDs to color based on alliance

#### Test

This command activates LEDs in a flashing pattern at 2 hz and sets LEDs to color based on alliance

### Command Pattern (highest priorities)

#### Shoot

This command activates LEDs in a chaser pattern and sets LEDs to color based on alliance

#### Overide Indexer

This command flashes LEDs off until command ends
