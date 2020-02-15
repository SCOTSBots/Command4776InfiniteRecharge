/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
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
import frc.robot.Tools.ColorTools;
import frc.robot.Tools.ColorTools.BaseColor;
import frc.robot.commands.CalibrateColorSliceCounts;
import frc.robot.commands.ControlPanelGotoColor;
import frc.robot.commands.ControlPanelRotate;

/**
 * The ControlPanelController subsystem controls the wheel rotator motor, color sensor, and servo rotator.
 * <p>Together, these three systems work to control the control panel as intended.
 * <p><b>Note:</b> The order of the colors on the panel is <b>Red, Green, Blue, Yellow
 */
public class ControlPanelController extends SubsystemBase {
  //Note: reading the color wheel works well, but for sensing balls in the intake, use Blue and Proximity.
  CANSparkMax m_wheelRotatorMotor; //the wheel rotates the control panel
  CANSparkMax m_controllerRotatorMotor; //Rotates the mechanism to put it on the control panel
  CANEncoder m_wheelRotatorEncoder;
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
      m_wheelRotatorMotor = new CANSparkMax(ControlPanelConstants.kWheelRotatorMotorPort, MotorType.kBrushless);
      m_wheelRotatorEncoder = m_wheelRotatorMotor.getEncoder();

      m_controllerRotatorMotor = new CANSparkMax(ControlPanelConstants.kControlPanelRotatorMotorPort, MotorType.kBrushless);
      
      m_colorSensor = new ColorSensorV3(ControlPanelConstants.kColorSensorPort);
      m_colorMatcher = new ColorMatch();
      m_colorMatcher.addColorMatch(ControlPanelConstants.kBlueTarget);
      m_colorMatcher.addColorMatch(ControlPanelConstants.kGreenTarget);
      m_colorMatcher.addColorMatch(ControlPanelConstants.kRedTarget);
      m_colorMatcher.addColorMatch(ControlPanelConstants.kYellowTarget);

      ShuffleboardHelper.addColorSensor("Color Sensor", this);

      ControlPanelGotoColor gotoYellow = new ControlPanelGotoColor(this, BaseColor.Yellow);
      gotoYellow.setName("Goto Yellow");
      ControlPanelGotoColor gotoRed = new ControlPanelGotoColor(this, BaseColor.Red);
      gotoRed.setName("Goto Red");
      ControlPanelGotoColor gotoBlue = new ControlPanelGotoColor(this, BaseColor.Blue);
      gotoBlue.setName("Goto Blue");
      ControlPanelGotoColor gotoGreen = new ControlPanelGotoColor(this, BaseColor.Green);
      gotoGreen.setName("Goto Green");
      CalibrateColorSliceCounts ccsc = new CalibrateColorSliceCounts(this);
      ccsc.setName("Calibrate Slices");
      ControlPanelRotate rotate = new ControlPanelRotate(this);
      rotate.setName("Rotate 3 to 5");

      ShuffleboardHelper.AddOutput(this, "Control CP Wheel ("+ControlPanelConstants.kWheelRotatorMotorPort+")",
       -1, 1, this::setWheelSpeed, gotoYellow, gotoRed, gotoBlue, gotoGreen, ccsc, rotate);
    }
  }
  public BaseColor getBaseColorOnSensor() {
    if (!ControlPanelConstants.kHasControlPanel) return BaseColor.Unknown;
    Color detectedColor = m_colorSensor.getColor();
      /**
       * Run the color match algorithm on our detected color
       */
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

      if (match.color == ControlPanelConstants.kBlueTarget) {
        return ColorTools.robotColorToSensorColor(BaseColor.Blue);
      } else if (match.color == ControlPanelConstants.kRedTarget) {
        return ColorTools.robotColorToSensorColor(BaseColor.Red);
      } else if (match.color == ControlPanelConstants.kGreenTarget) {
        return ColorTools.robotColorToSensorColor(BaseColor.Green);
      } else if (match.color == ControlPanelConstants.kYellowTarget) {
        return ColorTools.robotColorToSensorColor(BaseColor.Yellow);
      } else {
        return BaseColor.Unknown;
      }
  }
  public Color getColor() {
    if (ControlPanelConstants.kHasControlPanel) {
      return m_colorSensor.getColor();
    }
    else {
      return null;//ControlPanelConstants.kRedTarget;
    }
  }
  public ColorMatchResult getEstimatedColor() {
    if (ControlPanelConstants.kHasControlPanel) {
      return m_colorMatcher.matchClosestColor(m_colorSensor.getColor());
    }
    else {
      return null;//ControlPanelConstants.kRedTarget;
    }
  }

  public void setWheelSpeed(double speed) {
    m_wheelRotatorMotor.set(speed);
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
  public double getCounts() {
    return m_wheelRotatorEncoder.getPosition();
  }
  public void power(double speed) {
    if (m_wheelRotatorMotor != null)
      m_wheelRotatorMotor.set(speed);
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
