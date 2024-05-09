package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hook;
import frc.robot.subsystems.Loader;
import frc.robot.commands.swervedrive.superStructure.AimAtTarget;
import frc.robot.commands.swervedrive.superStructure.ArmCommand;
import frc.robot.commands.swervedrive.superStructure.DetectAprilTagsCommand;
import frc.robot.commands.swervedrive.superStructure.HookCommand;
import frc.robot.commands.swervedrive.superStructure.LoaderCommand;
import frc.robot.commands.swervedrive.superStructure.shootLoadCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.swervedrive.superStructure.shooterCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/"));
  private final SendableChooser<Command> autoChooser;

  XboxController driverXbox = new XboxController(0);
  XboxController shooterXbox = new XboxController(1);

  Trigger backButton = new JoystickButton(driverXbox, XboxController.Button.kBack.value);
  Trigger startButton = new JoystickButton(driverXbox, XboxController.Button.kStart.value);

  Trigger resetHook = backButton.and(startButton);

  public Arm m_arm = new Arm();
  public Shooter shooter = new Shooter();
  public Loader loader = new Loader();
  public Hook hook = new Hook();

  public RobotContainer()
  {

    NamedCommands.registerCommand("ArmShoot", new ArmCommand(m_arm, -12.0));
    NamedCommands.registerCommand("ArmLoad", new ArmCommand(m_arm, -8.0));
    NamedCommands.registerCommand("ArmHome", new ArmCommand(m_arm, 0.0));
    NamedCommands.registerCommand("ShootLaunch", new shooterCommand(shooter, 0.8));
    NamedCommands.registerCommand("LoadShooter", new shooterCommand(shooter, -0.8));
    NamedCommands.registerCommand("LoadLaunch", new LoaderCommand(loader, -1));
    NamedCommands.registerCommand("Loader", new LoaderCommand(loader, 1));
    NamedCommands.registerCommand("Shooting", new shootLoadCommand(shooter, loader, 1, -1));

    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    drivebase.setDefaultCommand(
        driveFieldOrientedDirectAngle);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Selection", autoChooser);
  }

  private void configureBindings()
  {
    new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, XboxController.Button.kY.value).onTrue(new HookCommand(hook, 360));
    new JoystickButton(driverXbox, XboxController.Button.kX.value).onTrue(new HookCommand(hook, 0));
    resetHook.onTrue(new HookCommand(hook, -360));

    new JoystickButton(shooterXbox, XboxController.Button.kY.value).onTrue(new ArmCommand(m_arm, -12));
    new JoystickButton(shooterXbox, XboxController.Button.kB.value).onTrue(new ArmCommand(m_arm, -7.0));
    new JoystickButton(shooterXbox, XboxController.Button.kA.value).onTrue(new ArmCommand(m_arm, -0.0));
    /* new JoystickButton(shooterXbox, XboxController.Button.kLeftBumper.value).whileTrue(new shooterCommand(shooter, 1)); */
    new JoystickButton(shooterXbox, XboxController.Button.kRightBumper.value).whileTrue(new shooterCommand(shooter, -1));
    /* new JoystickButton(shooterXbox, XboxController.Button.kA.value).whileTrue(new LoaderCommand(loader, -1)); */
    new JoystickButton(shooterXbox, XboxController.Button.kRightBumper.value).whileTrue(new LoaderCommand(loader, 0.3));
    new JoystickButton(shooterXbox, XboxController.Button.kLeftBumper.value).onTrue(new shootLoadCommand(shooter, loader, 1, -1));
    /* new JoystickButton(shooterXbox, XboxController.Button.kLeftBumper.value).whileTrue(new DetectAprilTagsCommand()); */
    new JoystickButton(shooterXbox, XboxController.Button.kA.value).whileTrue(new AimAtTarget(drivebase, m_arm, ()->MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), ()->MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)));
  }

  public Command getAutonomousCommand()
  {
    /* return drivebase.getAutonomousCommand("RotateFromSpeaker", true); */
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}