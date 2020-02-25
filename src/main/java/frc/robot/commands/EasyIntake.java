/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class EasyIntake extends CommandBase {
  Intake intake;
  /**
   * Creates a new EasyIntake.
   */
  public EasyIntake(Intake intake) {
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.ballInShooter()) {
      
      intake.powerConveyor(0);
    }
    else {
    
      int status = intake.getBallsInRobot();
      boolean currentBall = intake.ballInIntake();
      intake.powerIntake(0.7);
      if (status <= 0) {
        intake.powerConveyor(0.7);
      }
      else {
        if (currentBall) {
          intake.powerConveyor(0.5);
        }
        else {
          intake.powerConveyor(0.3);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
