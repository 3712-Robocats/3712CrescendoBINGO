package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Crescendo;

public class Hook extends SubsystemBase {

    private CANSparkMax hookMotor;
    private SparkPIDController hookController;
    private RelativeEncoder hookEncoder;

    private static final double MIN_ENCODER_COUNT = -360.0;
    private static final double MAX_ENCODER_COUNT = 360.0;

    public Hook() {
        hookMotor = new CANSparkMax(Crescendo.Shooter.HOOK_ID, MotorType.kBrushless);
        hookMotor.restoreFactoryDefaults();
        hookMotor.setSmartCurrentLimit(20);
        hookMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        hookController = hookMotor.getPIDController();
        hookEncoder = hookMotor.getEncoder();

        hookController.setP(1.0);
        hookController.setI(0);
        hookController.setD(0);
        hookController.setFF(0.05);
        hookController.setIZone(0);
        hookController.setOutputRange(-1, 1);
    }

    public void raiseLowerHook(double targetPosition) {
        double clampedAngle = Math.max(MIN_ENCODER_COUNT, Math.min(targetPosition, MAX_ENCODER_COUNT));

        hookController.setReference(clampedAngle, CANSparkMax.ControlType.kPosition);
    }

    public double getCurrentPosition() {
        return hookEncoder.getPosition();
    }

    public void stopMotor() {
      hookMotor.set(0);
    }
    
    @Override
    public void periodic() {
      double currentEncoderCounts = hookEncoder.getPosition();
      SmartDashboard.putNumber("HOOK Raw Encoder Counts", currentEncoderCounts);
    }
}
