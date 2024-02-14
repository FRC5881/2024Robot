package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.PWM;

public class Lights {

    AddressableLED led;

    public enum color {

        /*
         * sets lights to FF0000
         * used for red alliance
         */
        red,

        /*
         * sets lights to 00FF00
         * used for testing + show
         */

        green,

        /*
         * sets lights to 0000FF
         * used for blue alliance
         */
        blue,
    }

    public enum pattern {

        /*
         * this command activates LEDs in a breathing strobe pattern
         * sets LEDs to color based on alliance
         */
        auto,

        /*
         * This command activates LEDs in a solid pattern
         * sets LEDs to color based on alliance
         */
        teleop,

        /*
         * This command activates LEDs in a flashing pattern at 2 hz
         * sets LEDs to color based on alliance
         */
        test,

        /*
         * This command activates LEDs in a chaser pattern
         * sets LEDs to color based on alliance
         */
        shoot,

        /*
         * This command flashes LEDs off for acertain amout of time until command ends
         */
        override,
    }

}
