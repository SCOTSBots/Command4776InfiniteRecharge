/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class LoadNextBall extends CommandBase {
  Intake intake;
  boolean hadBall;
  double oldPosition;
  DoubleSupplier override;
  double off;

  /**
   * Creates a new LoadNextBall.
   */
  public LoadNextBall(Intake intake, DoubleSupplier overrideInput, double offset) {
    addRequirements(intake);
    this.intake = intake;
    override = overrideInput;
    off = offset;
    //Shuffleboard.getTab("Intake").addString("Target", ()->{return "Target: "+oldPosition+", At: "+newPos;});
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hadBall = intake.ballInIntake();
    oldPosition = intake.getConveyorPosition() - 1000;
    fifth = false;
  }
  boolean fifth;
  double newPos;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intake.powerConveyor(1);
    boolean currentBall = intake.ballInIntake();
    int status = intake.getBallsInRobot();
    double power = 0;
    newPos = intake.getConveyorPosition();
    if (status == 4) {
      if ((oldPosition + 45.0) < newPos) {
        power = 1;
      }
      else {
        power = 0;
      }
      if (currentBall) {
        oldPosition = newPos;
      }
    }
    else if (status == 5) {
      if ((oldPosition + 15.0) > newPos) {
        power = 1;
      }
      else {
        power = 0;
      }
    }
    else {
      if (currentBall && !hadBall) {
        oldPosition = newPos;
      }
      if (currentBall || ((status!=0) && (oldPosition + off) > newPos)) {
        power = 1;
      }
      else {
        power = 0;
      }
    }
    if (!IntakeConstants.kHasIntakeColorSensor) power = override.getAsDouble();
    intake.powerConveyor(power);
    hadBall = currentBall;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.powerConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
