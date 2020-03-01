/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class AutoSweepTurret extends CommandBase {
  Shooter shooter;
  double target;
  double initialPosition;
  /**
   * Creates a new AutoSweepTurret.
   */
  public AutoSweepTurret(Shooter shooter, double start, double target) {
    this.shooter = shooter;
    this.target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPosition = shooter.getTurretPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Start sweep
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    target = shooter.TryToAim(initialPosition, target);
    System.out.println(("From "+initialPosition+" to "+ target));
    return target < -500;
  }
}
