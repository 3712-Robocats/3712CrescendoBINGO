package frc.robot.commands.swervedrive.superStructure;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class Move extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private double startPositionY;
    private final double targetDistance = 1.0; // 1 meter
    private int stage = 0;
    private final double tolerance = 0.1;

    public Move(SwerveSubsystem swerveSubsYstem) {
        this.swerveSubsystem = swerveSubsYstem;
        addRequirements(swerveSubsYstem);
    }

    @Override
    public void initialize() {
        startPositionY = swerveSubsystem.getPose().getTranslation().getY();
        stage = 0;
    }

    @Override
    public void execute() {
        double currentPositionY = swerveSubsystem.getPose().getTranslation().getY();
        double distanceTraveled = currentPositionY - startPositionY; // Calculate the distance traveled
        switch (stage) {
            case 0: // Move left 1 meter
                if (distanceTraveled > -targetDistance) { // Check if distance traveled is less than -(1 meter + tolerance)
                    swerveSubsystem.drive(new Translation2d(0, -0.5), 0, false); // Negative Y for left
                } else {
                    startPositionY = currentPositionY; // Update startPositionY for the next movement
                    stage++;
                }
                break;
            case 1: // Move right 2 meters
                if (distanceTraveled < targetDistance * 2 + tolerance) { // Check if distance traveled is less than (2 meters + tolerance)
                    swerveSubsystem.drive(new Translation2d(0, -0.5), 0, false); // Positive Y for right
                } else {
                    startPositionY = currentPositionY; // Update startPositionY for the next movement
                    stage++;
                }
                break;
            case 2: // Move left 1 meter to return to start
                if (distanceTraveled > -(targetDistance + tolerance)) { // Check if distance traveled is less than -(1 meter + tolerance)
                    swerveSubsystem.drive(new Translation2d(0, -0.5), 0, false); // Negative Y for left
                } else {
                    swerveSubsystem.drive(new Translation2d(0, 0), 0, false); // Stop movement
                    stage++;
                }
                break;
        }
    }



    @Override
    public boolean isFinished() {
        return stage > 2;
    }
}
