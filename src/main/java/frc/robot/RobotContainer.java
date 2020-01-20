/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import frc.robot.commands.CheesyDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SampleAutonomousDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public enum RobotType{
    KOP,
    WC,
    Jeff
  }
  private final RobotType robot = RobotType.KOP;
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain(robot);
  private final Command m_autoCommand = new SampleAutonomousDrive(driveTrain);
  public static XboxController driverJoystick = new XboxController(0);
  NetworkTableEntry left_front_velocity;
  NetworkTableEntry left_back_velocity;
  NetworkTableEntry right_front_velocity;
  NetworkTableEntry right_back_velocity;
  NetworkTableEntry left_front_voltage;
  NetworkTableEntry left_back_voltage;
  NetworkTableEntry right_front_voltage;
  NetworkTableEntry right_back_voltage;
  NetworkTableEntry left_front_current;
  NetworkTableEntry left_back_current;
  NetworkTableEntry right_front_current;
  NetworkTableEntry right_back_current;
  NetworkTableEntry gyro;
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //Shuffleboard.getTab("Shooter").add(driveTrain.shooter);
    ShuffleboardLayout servoLayout = Shuffleboard.getTab("Servo").getLayout("Servo Control", BuiltInLayouts.kList).withSize(3, 5);
    NetworkTableEntry servoPosition = servoLayout.addPersistent("Servo Position", 0.0).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min",0,"max",1)).withSize(5, 2).withPosition(3, 2).getEntry();
    
    InstantCommand controlServo = new InstantCommand(()->driveTrain.set(servoPosition.getDouble(0.0)));
    controlServo.setName("Move Servo");
    servoLayout.add(controlServo);
    //Shuffleboard.getTab("Shooter").add(driveTrain.servo);
    driveTrain.setDefaultCommand(new CheesyDrive(driveTrain));
    
    //Set up Shuffleboard
    //Set up Driver Station Tab
    Shuffleboard.getTab("Drive").addPersistent("Max Speed", 1.0).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min",0,"max",1)).withSize(5, 2).withPosition(3, 2);
    //Set up sample NetworkEntry
    NetworkTableEntry ne = Shuffleboard.getTab("Test Tab").add("Pi", 3.14).getEntry();
    //Set up sample command list
    
    ShuffleboardLayout commands = Shuffleboard.getTab("Test Tab").getLayout("Command List", BuiltInLayouts.kList).withSize(3, 5)
    .withProperties(Map.of("Label position", "HIDDEN"));
    //Get the Network Table Entry that controls the LEDs on the limelight so we can turn them on/off
    NetworkTableEntry ledMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode");
    //Make a command that inverts the leds on the limelight
    InstantCommand c = new InstantCommand(()->ledMode.setDouble((ledMode.getDouble(0)==1)?0:1));
    c.setName("Toggle LEDs");
    commands.add(c);
    
    
    //commands.add(new InstantCommand(() -> System.out.println("The OTHER New Command Pressed!")));
    Shuffleboard.getTab("Test Tab").add(driveTrain);
    ShuffleboardLayout layout = Shuffleboard.getTab("Drive").getLayout("NEW Motor Speeds", BuiltInLayouts.kList).withSize(6, 8).withPosition(7, 3);
    
    left_front_velocity = layout.add("Left Front Velocity", 0).getEntry();
    left_back_velocity = layout.add("Left Back Velocity", 0).getEntry();
    right_front_velocity = layout.add("Right Front Velocity", 0).getEntry();
    right_back_velocity = layout.add("Right Back Velocity", 0).getEntry();
    left_front_voltage = layout.add("Left Front Voltage", 0).getEntry();
    left_back_voltage = layout.add("Left Back Voltage", 0).getEntry();
    right_front_voltage = layout.add("Right Front Voltage", 0).getEntry();
    right_back_voltage = layout.add("Right Back Voltage", 0).getEntry();
    left_front_current = layout.add("Left Front Current", 0).getEntry();
    left_back_current = layout.add("Left Back Current", 0).getEntry();
    right_front_current = layout.add("Right Front Current", 0).getEntry();
    right_back_current = layout.add("Right Back Current", 0).getEntry();
    gyro = layout.add("Gyro", 0).getEntry();
  }
  public void setData(){
    double[] speeds = driveTrain.getSpeeds_velocity();
    left_front_velocity.setNumber(speeds[0]);
    left_back_velocity.setNumber(speeds[1]);
    right_front_velocity.setNumber(speeds[2]);
    right_back_velocity.setNumber(speeds[3]);
    speeds = driveTrain.getSpeeds_voltage();
    left_front_voltage.setNumber(speeds[0]);
    left_back_voltage.setNumber(speeds[1]);
    right_front_voltage.setNumber(speeds[2]);
    right_back_voltage.setNumber(speeds[3]);
    speeds = driveTrain.getSpeeds_current();
    left_front_current.setNumber(speeds[0]);
    left_back_current.setNumber(speeds[1]);
    right_front_current.setNumber(speeds[2]);
    right_back_current.setNumber(speeds[3]);
    gyro.setNumber(driveTrain.getGyro());
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
