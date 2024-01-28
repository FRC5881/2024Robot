package frc.robot.utils;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PickerCommandFactory {
    private final SendableChooser<String> chooser = new SendableChooser<>();
    private final HashMap<String, Command> map;

    public PickerCommandFactory(String name, Command... commands) {
        map = new HashMap<>();
        for (Command c : commands) {
            map.put(c.getName(), c);
        }

        chooser.setDefaultOption(commands[0].getName(), commands[0].getName());
        for (int i = 1; i < commands.length; i++) {
            chooser.addOption(commands[i].getName(), commands[i].getName());
        }

        SmartDashboard.putData(name, chooser);
    }

    public Command build() {
        return Commands.select(map, chooser::getSelected);
    }
}
