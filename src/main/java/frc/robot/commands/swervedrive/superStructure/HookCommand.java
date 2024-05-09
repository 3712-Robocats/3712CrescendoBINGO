package frc.robot.commands.swervedrive.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hook;

public class HookCommand extends Command {
    private final Hook hook;
    private final double targetPosition;
    private static final double TOLERANCE = 1.0;

    public HookCommand(Hook hook, double targetPosition) {
        this.hook = hook;
        this.targetPosition = targetPosition;
        addRequirements(hook);
    }

    @Override
    public void initialize() {
        hook.raiseLowerHook(targetPosition);
    }

    @Override 
    public void end(boolean interrupted) {
        hook.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(hook.getCurrentPosition() - targetPosition) <= TOLERANCE;
    }
}
