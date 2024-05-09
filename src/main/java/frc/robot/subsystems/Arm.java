package frc.robot.subsystems;

import frc.robot.Constants.Crescendo.Shooter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final CANSparkMax leaderMotor;
  private final CANSparkMax followerMotor;
  private final SparkPIDController pidController;
  private final RelativeEncoder encoder;

  private static final double MIN_ENCODER_COUNT = -14.0; // Minimum rotation
  private static final double MAX_ENCODER_COUNT = 0.0; // Maximum rotation


  public Arm() {
    leaderMotor = new CANSparkMax(Shooter.ARM_MASTER, MotorType.kBrushless);
    followerMotor = new CANSparkMax(Shooter.ARM_FOLLOWER, MotorType.kBrushless);

    leaderMotor.restoreFactoryDefaults();
    followerMotor.restoreFactoryDefaults();

    leaderMotor.setIdleMode(IdleMode.kBrake);
  
    followerMotor.follow(leaderMotor, false);

    pidController = leaderMotor.getPIDController();
    encoder = leaderMotor.getEncoder();

    pidController.setP(0.1);
    pidController.setI(0);
    pidController.setD(0.1);
    pidController.setFF(0);
    pidController.setIZone(0);
    pidController.setOutputRange(-0.2, 0.1);
  }

  public void setTargetPosition(double targetPosition) {
    // Clamp the angle to be within the specified bounds
    /* UpdatePID(0.1,0,0.1); */
    double clampedAngle = Math.max(MIN_ENCODER_COUNT, Math.min(targetPosition, MAX_ENCODER_COUNT));

    
    /* double encoderCountsPerOutputRevolution = GEAR_RATIO * CPR;
    double encoderCountsForAngle = (clampedAngle / 360.0) * encoderCountsPerOutputRevolution; */
    pidController.setReference(clampedAngle, CANSparkMax.ControlType.kPosition);
  }

  public void UpdatePID(double p, double i, double d) {
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
  }

  public double getCurrentPosition() {
    return encoder.getPosition();
  }


  @Override
  public void periodic() {
    double currentEncoderCounts = encoder.getPosition();
    SmartDashboard.putNumber("ARM Raw Encoder Counts", currentEncoderCounts);
  }
}