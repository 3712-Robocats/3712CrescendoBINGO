package frc.robot.commands.swervedrive.superStructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Loader;

public class shootLoadCommand extends Command {
    private final Shooter shooter;
    private final Loader loader;
    private final double shooterSpeed;
    private final double loaderSpeed;
    private boolean loaderActive = false;
    private final Timer timer = new Timer();
    private static final double loaderDuration = 0.5; // Loader activate time in seconds
    
    public shootLoadCommand(Shooter shooter, Loader loader, double shooterSpeed, double loaderSpeed) {
        this.shooter = shooter;
        this.loader = loader;
        this.shooterSpeed = shooterSpeed;
        this.loaderSpeed = loaderSpeed;
        addRequirements(shooter, loader);
    }

    @Override
    public void initialize() {
        shooter.setMotorSpeed(shooterSpeed);
        timer.reset();
        timer.stop();
        loaderActive = false;
    }

    @Override
    public void execute() {
        if (!loaderActive && shooter.isAtFullSpeed()) {
            loader.loadLaunch(loaderSpeed);
            timer.start();
            loaderActive = true;
        }
        if (timer.hasElapsed(loaderDuration)) {
            loader.stopMotor();
        }
    }

    @Override
    public boolean isFinished() {
        return loaderActive && (timer.get() > loaderDuration);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
        loader.stopMotor();
        if (interrupted) {
            timer.stop();
        }
    }
}
