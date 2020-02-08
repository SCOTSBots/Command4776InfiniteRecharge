/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Tools.Pair;
import frc.robot.pixy.Pixy2CCC.Block;
import frc.robot.subsystems.ControlPanelController;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final ControlPanelController m_cpController = new ControlPanelController();
  // The driver's controller
  private final XboxController m_driverJoystick = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_manipulatorJoystick = new XboxController(OIConstants.kManipulatorControllerPort);
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
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
    
    //TODO: add this back: InstantCommand controlServo = new InstantCommand(()->driveTrain.set(servoPosition.getDouble(0.0)));
    //controlServo.setName("Move Servo");
    //servoLayout.add(controlServo);
    //Shuffleboard.getTab("Shooter").add(driveTrain.servo);
    m_driveTrain.setDefaultCommand(
      //new CheesyDrive(driveTrain)
      new RunCommand(
      ()->{
        Block b = m_driveTrain.getBiggestBlock();
        int pos = (b==null)?150:b.getX();
        double calculate = ((1.0*pos - 150.0)/150.0) / 1.0;
        double turn = m_driverJoystick.getAButton()?(
          calculate
        ):m_driverJoystick.getX(GenericHID.Hand.kRight);
        m_driveTrain.curvatureDrive(
          -m_driverJoystick.getY(GenericHID.Hand.kLeft), 
          turn, 
          m_driverJoystick.getBumper(GenericHID.Hand.kRight), (l,r)->{
            m_driveTrain.tankDriveVolts(l*10, r*10);
          });
    },m_driveTrain));
    m_cpController.setDefaultCommand(new RunCommand(()->{
      m_cpController.set(m_manipulatorJoystick.getY(GenericHID.Hand.kLeft));
    },m_cpController));
    //Set up Shuffleboard
    //Set up Driver Station Tab
    Shuffleboard.getTab("Drive").addPersistent("Max Speed", 1.0).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min",0,"max",1)).withSize(5, 2).withPosition(3, 2);
    //Set up sample NetworkEntry
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
    Shuffleboard.getTab("Test Tab").add(m_driveTrain);
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
   * @throws IOException
   */
  public Command getAutonomousCommand() throws IOException {
    // An ExampleCommand will run in autonomous
    return MultiRamseteCommands("one","two");
  }
  private SequentialCommandGroup MultiRamseteCommands(String... files) throws IOException {
    SequentialCommandGroup c = new SequentialCommandGroup();
    for (String s : files) {
        var data = EasyRamseteCommand(s);
        c.addCommands(data.getT1().beforeStarting(()->{
            System.out.println("Command \'"+s+"\' is now running at "+data.getT2().getInitialPose());
            m_driveTrain.resetOdometry(data.getT2().getInitialPose());
        }, m_driveTrain));
    }
    return c;
  }
  private Pair<Command, Trajectory> EasyRamseteCommand(String file) throws IOException {
    Trajectory jsonTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(
        "/home/lvuser/deploy/output/"+file+".wpilib.json"));
    System.out.println("EasyRamseteCommand loaded \'"+file+"\' successfully.");
    
    return new Pair<Command, Trajectory>(new RamseteCommand(
        jsonTrajectory,
        m_driveTrain::getPose,
        //disabledRamsete,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_driveTrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        m_driveTrain::tankDriveVolts,
        m_driveTrain
    ).andThen(() -> m_driveTrain.tankDriveVolts(0, 0)),jsonTrajectory);
  }
}
