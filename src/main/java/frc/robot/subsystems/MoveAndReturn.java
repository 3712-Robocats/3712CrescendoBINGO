package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MoveAndReturn extends SubsystemBase {
    private final SwerveSubsystem swerveSubsystem;
    private double initialPosition;

    public MoveAndReturn(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.initialPosition = swerveSubsystem.getPose().getTranslation().getY();
    }

    public double getPosition() {
        return swerveSubsystem.getPose().getTranslation().getY();
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        Translation2d translation = new Translation2d(xSpeed, ySpeed);
        swerveSubsystem.drive(translation, rot, fieldRelative);
    }

    public void stop() {
        // Stop all modules by setting their desired state to zero speed
        swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
    }

    public void resetPosition() {
        initialPosition = swerveSubsystem.getPose().getTranslation().getY();
    }

    public double getInitialPosition() {
        return initialPosition;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("BingoY", swerveSubsystem.getPose().getTranslation().getY());
    }
}

