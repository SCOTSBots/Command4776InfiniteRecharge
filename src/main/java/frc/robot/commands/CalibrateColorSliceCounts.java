/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelController;

public class CalibrateColorSliceCounts extends CommandBase {
  ControlPanelController cpc;
  double startCount;
  double deltaCounts;
  Color oldColor;
  /**
   * Creates a new CalibrateColorSliceCounts.
   */
  public CalibrateColorSliceCounts(ControlPanelController cpc) {
    this.cpc = cpc;
    addRequirements(cpc);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startCount = cpc.getCounts();
    oldColor = cpc.getEstimatedColor().color;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cpc.setWheelSpeed(0.25);
    Color newColor = cpc.getEstimatedColor().color;
    if (newColor != oldColor) {
      double currentCounts = cpc.getCounts();
      deltaCounts = currentCounts - startCount;
      startCount = currentCounts;
      SmartDashboard.putNumber("Delta Counts", deltaCounts);
    }
    oldColor = newColor;
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
