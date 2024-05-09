package frc.robot.commands.swervedrive.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Loader;

public class LoaderCommand extends Command {
    private final Loader loader;
    private final double setSpeed;

    public LoaderCommand(Loader loader, double setSpeed) {
        this.loader = loader;
        this.setSpeed = setSpeed;
        addRequirements(loader);
    }

    @Override
    public void execute() {
        loader.loadLaunch(setSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        loader.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
