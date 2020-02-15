/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Tools.ColorTools.BaseColor;
import frc.robot.Tools.ColorTools;
import frc.robot.subsystems.ControlPanelController;

public class ControlPanelGotoColor extends CommandBase {
  ControlPanelController cpc;
  boolean colorReady;
  boolean counterClockWise;
  double startPos;
  BaseColor targetColor;
  BaseColor oldColor;
  double power;
  /**
   * Creates a new ControlPanelGotoColor.
   */
  public ControlPanelGotoColor(ControlPanelController cpc, BaseColor color) {
    power = ControlPanelConstants.kGotoPositionPower;
    setName("Control to "+color);
    colorReady = false;
    this.cpc = cpc;
    addRequirements(cpc);
    this.targetColor = color;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorReady = false;
    oldColor = cpc.getBaseColorOnSensor();
    counterClockWise = power < 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cpc.setWheelSpeed(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!colorReady) {
        BaseColor currentColor = cpc.getBaseColorOnSensor();
        BaseColor usedToBe = ColorTools.rotateColor(currentColor, counterClockWise, true);
        if (currentColor == targetColor && (oldColor == currentColor || usedToBe == oldColor)) {
        startPos = cpc.getCounts();
        colorReady = true;
      }
      else {
      }
      if (ColorTools.rotateColor(oldColor, counterClockWise, false) == currentColor)
      oldColor = currentColor;
      return false;
    }
    else {
      return cpc.getCounts() - startPos > ControlPanelConstants.kSliceWidthCounts / 2;
    }
  }
}
