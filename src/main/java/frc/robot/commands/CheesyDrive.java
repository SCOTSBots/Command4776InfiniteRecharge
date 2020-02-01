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
  double oldOutput;
  double limitChange = 0.1;
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
    FullSpeed,
    SlewLimit;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (1 == 1) {
    Mode m = Mode.SlewLimit;
    switch (m){
      case Cheesy: {
         
//RobotContainer.driverJoystick.getRawAxis(1)
//RobotContainer.driverJoystick.getRawAxis(4)
          driveTrain.cheesyDrive(deadzone(-RobotContainer.driverJoystick.getRawAxis(1)),deadzone( 
          RobotContainer.driverJoystick.getRawAxis(4)), RobotContainer.driverJoystick.getRawButton(6));
        
    } break;
      case Turn: {
        driveTrain.setSpeedsTankDrive(-(RobotContainer.driverJoystick.getRawButton(1)?1:0),(RobotContainer.driverJoystick.getRawButton(1)?1:0));
      } break;
      case FullSpeed: {
        double NEWPOWER = (RobotContainer.driverJoystick.getRawButton(1)?0.95:0);

        double change = NEWPOWER - oldOutput;
      change = Math.max(-limitChange, Math.min(change, limitChange));//clamp change
      oldOutput += change;
        driveTrain.setSpeedsTankDrive(oldOutput,oldOutput);
      } break;
      case SlewLimit: {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -driveTrain.speedLimiter.calculate(deadzone(-RobotContainer.driverJoystick.getRawAxis(1)))
            * DriveTrain.maxSpeed;
        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot =
            -driveTrain.rotationLimiter.calculate(deadzone(RobotContainer.driverJoystick.getRawAxis(4)))
                * DriveTrain.kMaxAngularSpeed;

        driveTrain.drive(xSpeed, rot);
      } break;
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //driveTrain.cheesyDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
  public static double deadzone(double x){
    double min = 0.1;
    if (Math.abs(x) > min){
      return x;
    }else{
      return 0;
    }
  }
}
