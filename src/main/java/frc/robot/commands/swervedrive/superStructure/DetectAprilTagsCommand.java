package frc.robot.commands.swervedrive.superStructure;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DetectAprilTagsCommand extends Command {
    private final NetworkTable limelightTable;
    private NetworkTableEntry tv, tx, ty, ta;

    public DetectAprilTagsCommand() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
    }

    @Override
    public void initialize() {
        // Do not believe we need anything here unless switching pipelines
    }

    @Override
    public void execute() {
        boolean hasTarget = tv.getDouble(0) == 1;

        SmartDashboard.putBoolean("AprilTag Detected", hasTarget);

        if (hasTarget) {
            double xOffset = tx.getDouble(0.0);
            double yOffset = ty.getDouble(0.0);
            double area = ta.getDouble(0.0);

            SmartDashboard.putNumber("AprilTag X Offset", xOffset);
            SmartDashboard.putNumber("AprilTag Y Offset", yOffset);
            SmartDashboard.putNumber("AprilTag", area);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}