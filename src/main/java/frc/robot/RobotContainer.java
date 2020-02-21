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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.ColorShim;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Tools.MathTools;
import frc.robot.Tools.DataTools.Pair;
import frc.robot.commands.LoadNextBall;
import frc.robot.pixy.Pixy2CCC.Block;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControlPanelController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final LEDController m_leds = new LEDController();

  // The driver's controller
  private final XboxController m_driverJoystick = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_manipulatorJoystick = new XboxController(OIConstants.kManipulatorControllerPort);
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    System.out.println("Hello and Welcome to Infinite Recharge! Starting up robot \""+Constants.RobotName+"\"!");
    // Configure the button bindings
    configureButtonBindings();

    //Set up Shuffleboard
    //Set up Driver Station Tab
    //Shuffleboard.getTab("Drive").addPersistent("Max Speed", 1.0).withWidget(BuiltInWidgets.kNumberSlider)
    //  .withProperties(Map.of("min",0,"max",1)).withSize(5, 2).withPosition(3, 2);
    //Set up sample NetworkEntry
    //Set up sample command list
    
    ShuffleboardLayout commands = Shuffleboard.getTab("Test Tab").getLayout("Limelight Command List", BuiltInLayouts.kList).withSize(3, 5)
    .withProperties(Map.of("Label position", "HIDDEN"));
    //Get the Network Table Entry that controls the LEDs on the limelight so we can turn them on/off
    NetworkTableEntry ledMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode");
    //Get the Network Table Entry that controls the camera stream output see we can change the PnP
    NetworkTableEntry stream = NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream");
    //Make a command that inverts the leds on the limelight
    InstantCommand LEDs = new InstantCommand(()->ledMode.setDouble((ledMode.getDouble(0)==1)?3:1));
    InstantCommand CameraModes = new InstantCommand(()->stream.setDouble((stream.getDouble(0)==2)?1:2));
    LEDs.setName("LED Mode");
    CameraModes.setName("Camera Mode");
    commands.add(LEDs);
    commands.add(CameraModes);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (DriveConstants.kHasDriveTrain) {
      Runnable pixy = ()->{
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
      };
      Runnable cheesyDrive = ()->{
        double turn = m_driverJoystick.getX(GenericHID.Hand.kRight);
        m_driveTrain.curvatureDrive(
          -m_driverJoystick.getY(GenericHID.Hand.kLeft), 
          turn, 
          m_driverJoystick.getBumper(GenericHID.Hand.kRight), (l,r)->{
            m_driveTrain.tankDriveVolts(l*10, r*10);
          });
      };
      switch (OIConstants.teleop) {
        case CheesyDrive:  
          m_driveTrain.setDefaultCommand(new RunCommand(cheesyDrive,m_driveTrain));
          break;
        case PixyDrive:
          m_driveTrain.setDefaultCommand(new RunCommand(pixy,m_driveTrain));
          break;
      }
    }
    if (ControlPanelConstants.kHasControlPanel) {
      m_cpController.setDefaultCommand(new RunCommand(()->{
        //m_cpController.set(m_manipulatorJoystick.getY(GenericHID.Hand.kLeft));
      },m_cpController));
    }
    if (IntakeConstants.kHasIntake) {
      new JoystickButton(m_manipulatorJoystick, Button.kX.value).whenPressed(()->{m_intake.setFlipper(false);});
      
      //new JoystickButton(m_manipulatorJoystick, Button.kStart.value).whenPressed(()->{m_intake.resetFlipper(0);});
      //new JoystickButton(m_manipulatorJoystick, Button.kY.value).whenPressed(()->{m_intake.powerFlipper(m_manipulatorJoystick.getY(GenericHID.Hand.kLeft));});
      new JoystickButton(m_manipulatorJoystick, Button.kBack.value).whenPressed(m_intake::toggleFlipper);
      LoadNextBall lnb = new LoadNextBall(m_intake);
      new JoystickButton(m_manipulatorJoystick, Button.kBumperLeft.value).whenPressed(()->{
        m_intake.setFlipper(true);
        lnb.schedule();
      }).whenReleased(()->{
        //m_intake.setFlipper(false);
        m_intake.powerIntake(0);
        lnb.cancel();
      }).whileActiveContinuous(()->{
        m_intake.powerIntake(0.7);
      });
      new JoystickButton(m_manipulatorJoystick, Button.kBumperRight.value).whenReleased(()->{
        m_intake.powerIntake(0);
      }).whileActiveContinuous(()->{
        m_intake.powerIntake(-0.7);
      });
      new JoystickButton(m_manipulatorJoystick, Button.kB.value).whenPressed(()->{
        m_intake.setBallsInRobot(0);
      });
      
      m_intake.setDefaultCommand(new RunCommand(()->{
        //if (m_manipulatorJoystick.getYButton()) {
          double speed = MathTools.deadzone( m_manipulatorJoystick.getTriggerAxis(GenericHID.Hand.kLeft) );
          double aspeed = MathTools.deadzone( m_manipulatorJoystick.getTriggerAxis(GenericHID.Hand.kRight) );
          boolean ready = m_shooter.atSpeed();
          //m_intake.powerConveyor(ready?1:0);
          m_intake.powerConveyor(ready?1:speed-aspeed);
        
      },m_intake));
    }
    if (ShooterConstants.kHasShooter) {
      m_shooter.setDefaultCommand(new RunCommand(()->{
        double speed = -m_manipulatorJoystick.getY(GenericHID.Hand.kRight);
        m_shooter.powerShooter(speed>0.5);
        // boolean down = m_manipulatorJoystick.getAButtonPressed();
        // boolean up = m_manipulatorJoystick.getStartButtonPressed();
        // m_shooter.setSpeed(up?0:down?1:0.5);

        int pov = m_manipulatorJoystick.getPOV();
        System.out.println("POV: "+pov);
        // if (pov == 0) {

        // }
        // else if (pov == )
      },m_shooter));
    }
    if (LEDConstants.kHasLEDs) {
      new JoystickButton(m_driverJoystick, Button.kA.value).whenPressed(()->{
        m_leds.burstInput.disturb(0);
      });
      new JoystickButton(m_driverJoystick, Button.kB.value).whenPressed(()->{
        m_leds.burstInput.clearDisturbances();
      });
      /*int endGame = 15;
      int totalAuto = 10;
      int totalTeleop = 30;
      int delay = 100;
      m_leds.bar.setUpdator(()->{
        int t = 0;
        if (t++ > delay) {

        }
        boolean auto = DriverStation.getInstance().isAutonomous();
        double time = DriverStation.getInstance().getMatchTime();
        
        m_leds.bar.filledColor = auto?ColorShim.kYellow:time < endGame?time%0.5>0.25?ColorShim.kRed:ColorShim.kBlack:ColorShim.kGreen; 
        return auto?(time / totalAuto):(time / totalTeleop);//-m_driverJoystick.getY(GenericHID.Hand.kLeft);
      });*/
    }
    if (ShooterConstants.kHasShooter && DriveConstants.kHasDriveTrain) {
      //Override the driver's controls when the manipulator wants to turn the chassis while turning the turret
      new JoystickButton(m_manipulatorJoystick, Button.kStickLeft.value).whileHeld(()->{
        double turn = 10*m_manipulatorJoystick.getX(GenericHID.Hand.kLeft);
        m_driveTrain.tankDriveVolts(turn, -turn);
      }, m_driveTrain, m_shooter);
  
      //Add A button - aims turret
      new JoystickButton(m_manipulatorJoystick, Button.kA.value).whileHeld(()->{
        double chassisTurn = 10 * m_shooter.AutoAimAndShoot();
        m_driveTrain.tankDriveVolts(chassisTurn, -chassisTurn);
      },m_shooter, m_driveTrain);
    }
    if (ClimberConstants.kHasClimber) {
      m_climber.setDefaultCommand(new RunCommand(()->{
        double up = m_driverJoystick.getTriggerAxis(GenericHID.Hand.kLeft);
        double down = m_driverJoystick.getTriggerAxis(GenericHID.Hand.kRight);
        m_climber.set(up-down);
      }, m_climber));
    }
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() throws IOException {
    // An ExampleCommand will run in autonomous
    //return MultiRamseteCommands("CircleRight");
    return null;
    //return MultiRamseteCommands(Map.of("DirectTrenchAuto", new WaitCommand(3), "DirectTrenchPickup",new WaitCommand(3)));
    //return MultiRamseteCommands("DirectTrenchAuto","DirectTrenchPickup","one","three","superAuto");
  }
  SequentialCommandGroup MultiRamseteCommands (Map<String,Command> map) {
    SequentialCommandGroup c = new SequentialCommandGroup();
    map.forEach((json, cmd)->{
      try {
        c.addCommands(EasyRamseteCommand(json).getT1(), cmd);
      } catch (IOException e) {
        e.printStackTrace();
        System.out.println("There was an error reading the file \'"+json+".wpilib.json\'!!!!!");
      }
    });
    return c;
  }
  private SequentialCommandGroup MultiRamseteCommands(String... files) throws IOException {
    SequentialCommandGroup c = new SequentialCommandGroup();
    for (String s : files) {
        var data = EasyRamseteCommand(s);
        c.addCommands(data.getT1().beforeStarting(()->{
            System.out.println("Command \'"+s+"\' is now running at "+data.getT2().getInitialPose());
            m_driveTrain.resetOdometry(data.getT2().getInitialPose());
        }, m_driveTrain));
        c.addCommands(new WaitCommand(3));
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
