// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.superStructure;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.Crescendo.Shooter;
import frc.robot.subsystems.Arm;

public class AimAtTarget extends Command {
  private final SwerveSubsystem SwerveSub;
  private final Arm armSub;
  private DoubleSupplier translationX;
  private DoubleSupplier translationY;
  private double heading;
  private double lastGoodHeading;
  private double TOLERANCE = 0.0;


  public AimAtTarget(SwerveSubsystem s_SwerveSubsystem, Arm a_ArmSubsystem, DoubleSupplier translationX, DoubleSupplier translationY) {
    this.SwerveSub = s_SwerveSubsystem;
    this.armSub = a_ArmSubsystem;
    this.translationX = translationX;
    this.translationY = translationY;
    lastGoodHeading = 0;
    addRequirements(s_SwerveSubsystem, a_ArmSubsystem);
  }

  private double calculateDesiredArmPosition(double verticalAngle) {
    final double maxVerticalAngle = 0.0;
    final double minArmPosition = -14.0;
    final double maxArmPosition = 0.0;
    
    // Normalize the vertical angle to a range of [0, 1] where 0 is max downward angle and 1 is max upward angle
    double normalizedAngle = (verticalAngle + maxVerticalAngle) / (2 * maxVerticalAngle);
    
    // Map the normalized angle to the arm's position range
    double armPosition = normalizedAngle * (minArmPosition - maxArmPosition) + maxArmPosition;
    
    // Clamp the arm position to within its physical limits
    armPosition = Math.max(minArmPosition, Math.min(armPosition, maxArmPosition));
    
    return armPosition;
}


  @Override
  public void initialize() {
    armSub.UpdatePID(0.5, 0.0, 0.0);
    
    /* SwerveSub.setVisionTargetID(7); */
  }

  @Override
  public void execute() {
    if(SwerveSub.isValidVisionTarget()){
      /* armSub.UpdatePID(0.005, 0.0, 0.0); */
      heading = -SwerveSub.getVisionAngle()/90;
      double verticleAngle = SwerveSub.getVsionTY();
      double desiredArmPosition = calculateDesiredArmPosition(verticleAngle);
      double currentPosition = armSub.getCurrentPosition();
      double YOffset = desiredArmPosition - 7.7;
      if(Math.abs(currentPosition - desiredArmPosition) > TOLERANCE) {
        armSub.setTargetPosition(YOffset);
        System.out.println(desiredArmPosition);
      }
    }else{
      /* System.out.println("Warning: Swerve Aim: Lost Target!"); */
      heading = 0;
    }
    lastGoodHeading = heading;


    SwerveSub.getSwerve().drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * SwerveSub.getSwerve().getMaximumVelocity(),
    Math.pow(translationY.getAsDouble(), 3) * SwerveSub.getSwerve().getMaximumVelocity()),
    heading * SwerveSub.getSwerve().getMaximumAngularVelocity(),
true,
false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.UpdatePID(0.1, 0, 0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}