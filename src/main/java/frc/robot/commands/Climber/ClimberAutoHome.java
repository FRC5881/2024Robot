package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberAutoHome extends Command {
    private final ClimberSubsystem climber;

    public ClimberAutoHome(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);

    }

    @Override
    public void initialize() {
        climber.retract();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return climber.isStopped();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
        if (!interrupted) {
            climber.zero();
        }
    }
}
