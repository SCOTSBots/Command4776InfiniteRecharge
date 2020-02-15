/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Tools.ColorTools.BaseColor;
import frc.robot.Tools.ColorTools;
import frc.robot.subsystems.ControlPanelController;

public class ControlPanelRotate extends CommandBase {
  ControlPanelController cpc;
  int slices;
  BaseColor oldColor;
  double power;
  double minRotations = 25;
  double maxRotations = 39;
  boolean counterClockWise;
  /**
   * Creates a new ControlPanelRotate.
   */
  public ControlPanelRotate(ControlPanelController cpc) {
    this.cpc = cpc;
    addRequirements(cpc);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    slices = 0;
    power = ControlPanelConstants.kRotatePower;
    cpc.set(power);
    oldColor = cpc.getBaseColor();
    counterClockWise = power < 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    BaseColor currentColor = cpc.getBaseColor();
    BaseColor usedToBe = ColorTools.rotateColor(currentColor, counterClockWise, true);
    if (usedToBe == oldColor && currentColor != oldColor) {
      slices++;
    }
    oldColor = currentColor;
    SmartDashboard.putNumber("Slices", slices);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cpc.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return slices > minRotations && slices < maxRotations;
  }
}
