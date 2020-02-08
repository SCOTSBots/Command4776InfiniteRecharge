/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShuffleboardHelper;
import frc.robot.Constants.ControlPanelConstants;

/**
 * The ControlPanelController subsystem controls the wheel rotator motor, color sensor, and servo rotator.
 * <p>Together, these three systems work to control the control panel as intended.
 * <p><b>Note:</b> The order of the colors on the panel is <b>Red, Green, Blue, Yellow
 */
public class ControlPanelController extends SubsystemBase {
  //Note: reading the color wheel works well, but for sensing balls in the intake, use Blue and Proximity.
  CANSparkMax m_wheelRotator;
  ColorSensorV3 m_colorSensor;
  ColorMatch m_colorMatcher;
  int rotations;
  ColorMatchResult oldColor;

  /**
   * Creates a new ControlPanelController.
   */
  public ControlPanelController() {
    rotations = 0;
    if (ControlPanelConstants.kHasControlPanel) {
      //m_wheelRotator = new CANSparkMax(ControlPanelConstants.kWheelRotatorMotorPort, MotorType.kBrushless);
      m_colorSensor = new ColorSensorV3(ControlPanelConstants.kColorSensorPort);
      m_colorMatcher = new ColorMatch();
      m_colorMatcher.addColorMatch(ControlPanelConstants.kBlueTarget);
      m_colorMatcher.addColorMatch(ControlPanelConstants.kGreenTarget);
      m_colorMatcher.addColorMatch(ControlPanelConstants.kRedTarget);
      m_colorMatcher.addColorMatch(ControlPanelConstants.kYellowTarget);

      ShuffleboardHelper.addColorSensor("Color Sensor", this);
    }
  }

  public Color getColor() {
    if (ControlPanelConstants.kHasControlPanel) {
      return m_colorSensor.getColor();
    }
    else {
      return ControlPanelConstants.kRedTarget;
    }
  }

  @Override
  public void periodic() {
    if (ControlPanelConstants.kHasControlPanel) {
      // This method will be called once per scheduler run
      Color detectedColor = m_colorSensor.getColor();
      /**
       * Run the color match algorithm on our detected color
       */
      String colorString = "Error";
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

      if (match.color == ControlPanelConstants.kBlueTarget) {
        colorString = "Blue";
      } else if (match.color == ControlPanelConstants.kRedTarget) {
        colorString = "Red";
      } else if (match.color == ControlPanelConstants.kGreenTarget) {
        colorString = "Green";
      } else if (match.color == ControlPanelConstants.kYellowTarget) {
        colorString = "Yellow";
      } else {
        colorString = "Unknown";
      }

      /**
       * Open Smart Dashboard or Shuffleboard to see the color detected by the 
       * sensor.
       */
      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Confidence", match.confidence);
      SmartDashboard.putString("Detected Color", colorString);
      SmartDashboard.putNumber("Distance", m_colorSensor.getProximity());
    }
  }

  public void set(double speed) {
    if (m_wheelRotator != null)
      m_wheelRotator.set(speed);
  }
  public void setupColors() {
    m_colorMatcher = new ColorMatch();
    m_colorMatcher.addColorMatch(ControlPanelConstants.kBlueTarget);
    m_colorMatcher.addColorMatch(ControlPanelConstants.kGreenTarget);
    m_colorMatcher.addColorMatch(ControlPanelConstants.kRedTarget);
    m_colorMatcher.addColorMatch(ControlPanelConstants.kYellowTarget);
  }

  public int getMiniRotations() {
    if (!ControlPanelConstants.kHasControlPanel) return rotations;
    ColorMatchResult newColor = m_colorMatcher.matchClosestColor(m_colorSensor.getColor());
    if (oldColor == null) {

    }
    oldColor = newColor;
    return rotations;
  }
}
