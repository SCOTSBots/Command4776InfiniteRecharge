/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class LoadNextBall extends CommandBase {
  Intake intake;
  int stage;
  /**
   * Creates a new LoadNextBall.
   */
  public LoadNextBall(Intake intake) {
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.powerConveyor(1);
    // if (stage == 1) {
    //   intake.powerConveyor(0.5);
    // }
    // else {
    //   intake.powerConveyor(1);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.powerConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stage == 1 ^ intake.ballInIntake()) {
      stage++;
    }
    return stage>=2;
  }
}
