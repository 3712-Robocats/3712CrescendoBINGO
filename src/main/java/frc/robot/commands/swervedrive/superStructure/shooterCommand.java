package frc.robot.commands.swervedrive.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class shooterCommand extends Command {
    private final Shooter shooter;
    private final double setSpeed;

    public shooterCommand(Shooter shooter, double setSpeed) {
        this.shooter = shooter;
        this.setSpeed = setSpeed;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setMotorSpeed(setSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setMotorSpeed(0);
    }  

    @Override
    public boolean isFinished() {
        return false;
    } 
}
