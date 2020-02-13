/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class CheesyDrive extends CommandBase {
  DriveTrain driveTrain;
  /**
   * Creates a new CheesyDrive.
   */
  public CheesyDrive(DriveTrain newDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = newDriveTrain;
    addRequirements(driveTrain);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  enum Mode{
    Cheesy,
    Turn,
    FullSpeed;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Mode m = Mode.Turn;
    switch (m){
      case Cheesy: {
        driveTrain.cheesyDrive(-RobotContainer.driverJoystick.getRawAxis(1), RobotContainer.driverJoystick.getRawAxis(4), false);
    } break;
      case Turn: {
        driveTrain.setSpeedsTankDrive(-(RobotContainer.driverJoystick.getRawButton(1)?1:0),(RobotContainer.driverJoystick.getRawButton(1)?1:0));
      } break;
      case FullSpeed: {
        driveTrain.setSpeedsTankDrive((RobotContainer.driverJoystick.getRawButton(1)?1:0),(RobotContainer.driverJoystick.getRawButton(1)?1:0));
      } break;
      
      }
    }
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.cheesyDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
