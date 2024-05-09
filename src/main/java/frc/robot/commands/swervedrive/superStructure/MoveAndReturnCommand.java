package frc.robot.commands.swervedrive.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MoveAndReturn;

public class MoveAndReturnCommand extends Command {
  private final MoveAndReturn moveAndReturn;
  private final double distance;
  private boolean goingRight;

  public MoveAndReturnCommand(MoveAndReturn moveAndReturn, double distance, boolean goingRight) {
    this.moveAndReturn = moveAndReturn;
    this.distance = distance;
    this.goingRight = goingRight;
    addRequirements(moveAndReturn);
  }

  @Override
  public void initialize() {
    moveAndReturn.resetPosition();
  }

  @Override
  public void execute() {
    double currentPosition = moveAndReturn.getPosition();
    double targetPosition = moveAndReturn.getInitialPosition() + (goingRight ? distance : -distance);

    if (goingRight) {
      if (currentPosition < targetPosition) {
        moveAndReturn.drive(0.0, 0.5, 0.0, false);  // Move right
      } else {
        goingRight = false;
      }
    }

    if (!goingRight) {
      if (currentPosition > targetPosition) {
        moveAndReturn.drive(0.0, -0.5, 0.0, false);  // Move left
      } else {
        goingRight = true;
      }
    }
  }

  /* @Override
  public void end(boolean interrupted) {
    moveAndReturn.stop();
  } */

  /* @Override
  public boolean isFinished() {
    double currentPosition = moveAndReturn.getPosition();
    double initialPosition = moveAndReturn.getInitialPosition();
    return Math.abs(currentPosition - initialPosition) < 0.1;
  } */
}
