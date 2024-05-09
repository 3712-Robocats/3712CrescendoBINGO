package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Loader extends SubsystemBase {

    private static final int loaderID = 10;
    private CANSparkMax loader;
    private SparkPIDController loaderController;
    private RelativeEncoder loaderEncoder;
    private final double maxRPM = 60;

    public Loader() {
        loader = new CANSparkMax(loaderID, MotorType.kBrushless);
        loader.restoreFactoryDefaults();
        loaderController = loader.getPIDController();
        loaderEncoder = loader.getEncoder();

        loaderController.setP(0);
        loaderController.setI(0);
        loaderController.setD(0);
        loaderController.setFF(0.50);
        loaderController.setIZone(0);
        loaderController.setOutputRange(-1, 1);
    }

    public void loadLaunch(double speed) {
        double setPoint = speed*maxRPM;
        loaderController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    }

    public double getEncoderVelocity() {
        return loaderEncoder.getVelocity();
    }

    public void stopMotor() {
        loader.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LOADER Encoder Velocity", getEncoderVelocity());
    }
}
