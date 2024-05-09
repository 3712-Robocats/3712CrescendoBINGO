package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Crescendo;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {

    /* public static final String CANSparkBase = null; */
    private final CANSparkMax m_motor;
    private final CANSparkMax m_motor2;
    private final SparkPIDController m_pidController;
    private final RelativeEncoder m_encoder;

    private final double maxRPM = 5500;
    private final double RPMTolerance = 0.1 * maxRPM;

    public Shooter() {
        m_motor = new CANSparkMax(Crescendo.Shooter.SHOOTER_LEFT, MotorType.kBrushless);
        m_motor2 = new CANSparkMax(Crescendo.Shooter.SHOOTER_RIGHT, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        m_motor2.restoreFactoryDefaults();

        m_motor2.follow(m_motor, true);
        m_motor.setIdleMode(IdleMode.kCoast);
        m_motor2.setIdleMode(IdleMode.kCoast);

        m_motor.setSmartCurrentLimit(40);
        
        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();

        m_pidController.setP(0.0);
        m_pidController.setI(0);
        m_pidController.setD(0);
        m_pidController.setFF(0.05);
        m_pidController.setIZone(0);
        m_pidController.setOutputRange(-0.2, 1);
    }

    public void setMotorSpeed(double speed) {
        double getSpeed = speed * maxRPM;
        m_pidController.setReference(getSpeed, CANSparkMax.ControlType.kVelocity);
    }

    public double getEncoderVelocity() {
        return m_encoder.getVelocity();
    }

    public boolean isAtFullSpeed() {
        return Math.abs(m_encoder.getVelocity() - maxRPM) < RPMTolerance;
    }

    public void stopMotor() {
        m_motor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("SHOOTER Encoder Velocity", getEncoderVelocity());
    }

}
