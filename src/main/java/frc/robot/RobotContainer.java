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
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
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
import frc.robot.commands.EasyIntake;
import frc.robot.commands.LoadNextBall;
import frc.robot.pixy.Pixy2CCC.Block;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControlPanelController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static java.util.Map.entry;

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

  private final SendableChooser<CommandsToChoose> m_chooser = new SendableChooser<>();
  private final Command m_selectCommand;
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

    //Create Autonomous Commands
    try {
      CreateAutoCommandsForAutoSelector();
    } catch (IOException e) {
      System.out.println("Auto commands could not be created!");
      e.printStackTrace();
    }
    m_selectCommand = new SelectCommand(Map.ofEntries(
      entry(CommandsToChoose.Default, DefaultAuto),
      entry(CommandsToChoose.DirectTrench, DirectTrench),
      entry(CommandsToChoose.Wall, Wall),
      entry(CommandsToChoose.DirectRndzvs, DirectRndzvs),
      entry(CommandsToChoose.PickupMissedBalls, PickupMissedBalls),
      entry(CommandsToChoose.MidZone, MidZone),
      entry(CommandsToChoose.Test, Test),
      entry(CommandsToChoose.Shoot3, Shoot3),
      entry(CommandsToChoose.Shoot3Drive, Shoot3Drive),
      entry(CommandsToChoose.SuperTrench, SuperTrench)
      ), this::select);
    m_chooser.setDefaultOption("Default Auto", CommandsToChoose.Default);
    m_chooser.addOption("Direct Trench", CommandsToChoose.DirectTrench);
    m_chooser.addOption("Wall", CommandsToChoose.Wall);
    m_chooser.addOption("Direct Rendezvous", CommandsToChoose.DirectRndzvs);
    m_chooser.addOption("Pickup Missed Balls", CommandsToChoose.PickupMissedBalls);
    m_chooser.addOption("Mid Zone", CommandsToChoose.MidZone);
    m_chooser.addOption("Shoot3", CommandsToChoose.Shoot3);
    m_chooser.addOption("Shoot3Drive", CommandsToChoose.Shoot3Drive);
    m_chooser.addOption("SuperTrench", CommandsToChoose.SuperTrench);
    
    Shuffleboard.getTab("Auto").add(m_chooser);
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
        double turn = MathTools.deadzone(m_driverJoystick.getX(GenericHID.Hand.kRight));
        m_driveTrain.curvatureDrive(
          -MathTools.deadzone(m_driverJoystick.getY(GenericHID.Hand.kLeft)), 
          turn, 
          m_driverJoystick.getBumper(GenericHID.Hand.kRight), (l,r)->{
            //m_driveTrain.tankDriveVolts(l*10, r*10);
            m_driveTrain.normalTank(l, r);
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
      
      new JoystickButton(m_driverJoystick, Button.kX.value).whenPressed(()->
        IntakeConstants.kHasIntakeColorSensor = !IntakeConstants.kHasIntakeColorSensor);
      
        //new JoystickButton(m_manipulatorJoystick, Button.kStart.value).whenPressed(()->{m_intake.resetFlipper(0);});
      //new JoystickButton(m_manipulatorJoystick, Button.kY.value).whenPressed(()->{m_intake.powerFlipper(m_manipulatorJoystick.getY(GenericHID.Hand.kLeft));});
      //new JoystickButton(m_manipulatorJoystick, Button.kBack.value).whenPressed(m_intake::toggleFlipper);
      //Command lnb = new EasyIntake(m_intake);//new LoadNextBall(m_intake);
      Command lnb = new LoadNextBall(m_intake, ()->{
        double speed = MathTools.deadzone( 
        m_manipulatorJoystick.getTriggerAxis(GenericHID.Hand.kLeft));
      double aspeed = MathTools.deadzone( m_manipulatorJoystick.getTriggerAxis(GenericHID.Hand.kRight));
      return (speed-aspeed);},30);

      new JoystickButton(m_manipulatorJoystick, Button.kBumperLeft.value).whenPressed(()->{
        m_intake.setFlipper(true);
        //lnb.schedule();
      }).whenReleased(()->{
        //m_intake.setFlipper(false);
        m_intake.powerIntake(0);
        //lnb.cancel();
      }).whileActiveContinuous(()->{
        m_intake.powerIntake(0.7);//m_intake.ballInIntake()?0.3:0.7);
      });
      new JoystickButton(m_manipulatorJoystick, Button.kBumperRight.value).whenReleased(()->{
        m_intake.powerIntake(0);
      }).whileActiveContinuous(()->{
        m_intake.powerIntake(-1);
      });
      // new JoystickButton(m_manipulatorJoystick, Button.kB.value).whenPressed(()->{
      //   m_intake.setBallsInRobot(0);
      // });
      
      m_intake.setDefaultCommand(new RunCommand(()->{
          if (m_driverJoystick.getBumper(Hand.kLeft)) {
            m_intake.powerConveyor(0);
          }
          else if (m_manipulatorJoystick.getStickButton(Hand.kRight)) {
            boolean ready = m_shooter.atSpeed();
            m_intake.powerConveyor(ready?1:0);
          }
          else {
            
            double speed = MathTools.deadzone( m_manipulatorJoystick.getTriggerAxis(GenericHID.Hand.kLeft) );
            double aspeed = MathTools.deadzone( m_manipulatorJoystick.getTriggerAxis(GenericHID.Hand.kRight) );
          
            m_intake.powerConveyor(speed-aspeed);  
          }
          // Old Code:
          // double speed = MathTools.deadzone( m_manipulatorJoystick.getTriggerAxis(GenericHID.Hand.kLeft) );
          // double aspeed = MathTools.deadzone( m_manipulatorJoystick.getTriggerAxis(GenericHID.Hand.kRight) );
          // boolean ready = m_shooter.atSpeed();
          // m_intake.powerConveyor(ready?1:speed-aspeed);
        
      },m_intake));
    }
    
    if (ShooterConstants.kHasShooter) {
      new JoystickButton(m_manipulatorJoystick, Button.kY.value).whenPressed(m_shooter::toggleLimelight);
      new JoystickButton(m_driverJoystick, Button.kBumperLeft.value).whenPressed(new WaitUntilCommand(m_shooter::turretIntoClimbMode));
      new JoystickButton(m_driverJoystick, Button.kY.value).whenPressed(()->m_shooter.CalibrateTurret(0));
      m_shooter.setDefaultCommand(new RunCommand(()->{
        if (m_driverJoystick.getBumper(Hand.kLeft)) {
          m_shooter.turretIntoClimbMode();
        }
        else {
          if (m_manipulatorJoystick.getBButton()) {
            m_shooter.ZeroTurret();
          }
          else if (m_manipulatorJoystick.getAButton()) {
            m_shooter.AutoAimAndShoot();
          }
          else {
            double turn = MathTools.deadzone(m_manipulatorJoystick.getX(GenericHID.Hand.kLeft));
            m_shooter.powerTurret(turn);
            //m_shooter.disableLimelight();
          }
          double speed = -m_manipulatorJoystick.getY(GenericHID.Hand.kRight);
          //m_shooter.powerShooter(speed);
          if (speed < -0.7) {
            m_shooter.powerShooter(speed);
          }else {
            m_shooter.powerShooter(speed>0.5);
          }
        }
        int pov = m_manipulatorJoystick.getPOV();
        switch(pov) {
          case 0: {//UP DPAD
            m_shooter.hoodPosition(ShooterConstants.kHoodPositionZone2);
          }
            break;
          case 90: {//Right DPAD
            // m_shooter.setZoomPipeline(1);
            m_shooter.hoodPosition(ShooterConstants.kHoodPositionZone3);
          }
            break;
          case 180:{//DOWN DPAD
            m_shooter.hoodPosition(ShooterConstants.kHoodPositionZone4);
          }
          break;
          case 270: {//LEFT DPAD
            m_shooter.hoodPosition(ShooterConstants.kHoodPositionZone1);
          }
            break;
            default: {
              m_shooter.stopHood();
            }
        }
        //m_shooter.toggleSide(pov == 180);
        
        boolean down = m_manipulatorJoystick.getBackButton();
        boolean up = m_manipulatorJoystick.getStartButton();
        if (down) {
          //m_shooter.hoodPower(1);
        }
        else if (up) {
          //m_shooter.hoodPower(-1);
        }
        else {
          //m_shooter.hoodHoldPosition();
        }
      },m_shooter));
    }
    if (LEDConstants.kHasLEDs) {
      new JoystickButton(m_driverJoystick, Button.kA.value).whenPressed(()->{
        m_leds.burstInput.disturb(0);
      });
      new JoystickButton(m_driverJoystick, Button.kB.value).whenPressed(()->{
        m_leds.burstInput.clearDisturbances();
      });
      if (IntakeConstants.kHasIntake) {
        m_leds.ballCapacity.setUpdator(()->1.0*m_intake.getBallsInRobot()/5.0);
      }
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
    if (ClimberConstants.kHasClimber) {
      m_climber.setDefaultCommand(new RunCommand(()->{
        if (m_driverJoystick.getBumper(Hand.kLeft)) {
          double up = m_manipulatorJoystick.getTriggerAxis(GenericHID.Hand.kLeft);
          double down = m_manipulatorJoystick.getTriggerAxis(GenericHID.Hand.kRight);
          m_climber.set(MathTools.deadzone(up-down));
        }
        else {
          m_climber.set(0);
        }
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
    //return side();
    //return basicAuto();
    // return new LoadNextBall(m_intake);
    //return MultiRamseteCommands("straight").andThen(new WaitCommand(3)).andThen(BackwardsRamseteCommand("straight"));
    //return ResetBalls().andThen(StealAuto(), RapidShoot(-1));
    //return DirectTrench;
    return m_selectCommand;
    //return SafetyShoot(3,false).andThen(MaintainIntake(RamseteCommand("ForwardTrenchPickup")));
    //return new AutoSweepTurret(m_shooter, 0, 100);
    //return AutoIntake(m_intake);
    //return RapidShoot(3);
    //return MultiRamseteCommands(Map.of("DirectTrenchAuto", new WaitCommand(3), "DirectTrenchPickup",new WaitCommand(3)));
    //return MultiRamseteCommands("DirectTrenchAuto","DirectTrenchPickup","one","three","superAuto");
  }
  private void CreateAutoCommandsForAutoSelector() throws IOException {
    DefaultAuto = RamseteCommand("straight");
    //These are named exactly as they are in "Autonomous Strategy" on page 2 in the Google Drive.
    DirectTrench = SetShooterSpeeds(4500, 7000).andThen(CalibrateTurretPosition(-126).andThen(
      TimedShoot(3, false, 5),
      //was DirectTrenchPickupShoot3
      ((/*SudoTurret(0,0.3,*/MaintainIntake(RamseteCommand("ForwardTrenchPickup").deadlineWith(setZoom(2)))))
    ));
      //new AutoSweepTurret(m_shooter, 0, 90),
      //RapidShoot(1, true));
    Command c = RamseteCommand("TrenchButCool");
    DirectTrench = SetShooterSpeeds(3300, 7000).andThen(CalibrateTurretPosition(-126).andThen(
      TimedShoot(3, false, 5),
      //was DirectTrenchPickupShoot3
      ((/*SudoTurret(0,0.3,*/MaintainIntake(RamseteCommand("ForwardTrenchPickup").deadlineWith(setZoom(2)))))
    ));
    Shoot3 = TimedShoot(3, false, 7);
    Shoot3Drive = TimedShoot(3, false, 7).andThen(RamseteCommand("straight"));
    MidZone = CalibrateTurretPosition(-60).andThen(RamseteCommand("HP47m"),TimedShoot(3,true,5)
    //,MaintainIntake(RamseteCommand("MoveMid"))
    );
    

    Test = SafetyShoot(3,false, 7.).andThen(MaintainIntake(RamseteCommand("ForwardTrenchPickup")));
    Wall = new PrintCommand("Wall command ran.");
    DirectRndzvs = new PrintCommand("Rendezvous command ran.");
    MidZone = new PrintCommand("h");//Wall;//RamseteCommand("MoveMid").andThen(SafetyShoot(3, false, 5), RamseteCommand("MidToPickup"));
    PickupMissedBalls = new PrintCommand("PickupMissedBalls command ran.");
    SuperTrench = SetShooterSpeeds(3300, 7000).andThen(CalibrateTurretPosition(-126)).andThen(
      TimedShoot(3, false, 5)
      //, MaintainIntake(RamseteCommand("ForwardTrenchPickup").deadlineWith(TimedShoot(-1, false, 10), AutoAimer(90, 0.5)))
    );
  }
  public Command Test;
  public Command DefaultAuto;
  public Command Wall;
  public Command DirectTrench;
  public Command DirectRndzvs;
  public Command PickupMissedBalls;
  public Command MidZone;
  public Command Shoot3;
  public Command Shoot3Drive;
  public Command SuperTrench;
  Command AutoAimer(double targetPosition, double max) {
    Command c = new RunCommand(()->m_shooter.SudoTurretToPosition(targetPosition, max)).
    withInterrupt(()->Math.abs(m_shooter.getTurretPosition()-targetPosition) < 10);
    return c.andThen(new RunCommand(m_shooter::AutoAimAndShoot));
  }
  InstantCommand setZoom(int level) {
    return new InstantCommand(()->m_shooter.setZoomPipeline(level));
  }
  InstantCommand ResetBalls() {
    return new InstantCommand(()->m_intake.setBallsInRobot(0));
  }
  InstantCommand CalibrateTurretPosition(double targetPosition) {
    return new InstantCommand(()->m_shooter.CalibrateTurret(targetPosition));
  }
  InstantCommand SetShooterSpeeds(double targetSpeed, double pid) {
    return new InstantCommand(()->m_shooter.setSpeeds(targetSpeed, pid));
  }
  Command StealAuto() throws IOException {
    return MaintainIntake(RamseteCommand("straight").andThen(BackwardsRamseteCommand("backup")));
  }
  /**
   * This runs the intake while you are running your command.
   * @param cmd The command you want to run at he same time. Usually a RamseteCommand
   * @return the output command
   */
  Command MaintainIntake(Command cmd) {
    return new InstantCommand(()->m_intake.setFlipper(true)).andThen(cmd.deadlineWith(
      new LoadNextBall(m_intake, ()->m_intake.ballInShooter()?0.0:1.0, 40), new InstantCommand(()->m_intake.powerIntake(m_intake.ballInShooter()?0.3:0.7))
    )).andThen(new InstantCommand(()->{
      m_intake.setFlipper(false);
      m_intake.powerIntake(0);
    }));
  }
  Command SudoTurret(double targetPosition, double max, Command cmd) {
    return cmd.deadlineWith(new RunCommand(()->m_shooter.SudoTurretToPosition(targetPosition, max)).
    withInterrupt(()->{return Math.abs(m_shooter.getTurretPosition()-targetPosition) < 10;}));
  }
  @Deprecated
  ParallelCommandGroup xAutoIntake(Intake intake) {
    return new ParallelCommandGroup(new LoadNextBall(intake,()->0, 30), new RunCommand(()->{
      intake.powerIntake(0.7);
      intake.setFlipper(true);
    }).andThen(()->{
      intake.powerIntake(0);
      intake.setFlipper(false);
    }));
  }
  private Command side() throws IOException {
    SequentialCommandGroup c = new SequentialCommandGroup();
    Command b = RamseteCommand("UltraTwank").deadlineWith(disableDevices());
    // Command pickUpBalls = RamseteCommand("PICKUP BALLS").deadlineWith(
    //   new EasyIntake(m_intake), 
    //   new RunCommand(()->m_shooter.powerShooter(false)));
    //Command shoot = 
    //c.addCommands(RapidShoot(3),b,pickUpBalls,RapidShoot(0));//, pickUpBalls,RapidShoot());
    c.addCommands(b,RapidShoot(3, false));
    return c;
  }
  private Command basicAuto() throws IOException {
    SequentialCommandGroup c = new SequentialCommandGroup();
    Command b = RamseteCommand("Basic").deadlineWith(disableDevices());
    Command pickUpBalls = RamseteCommand("PICKUP BALLS").deadlineWith(
      new EasyIntake(m_intake), 
      new RunCommand(()->m_shooter.powerShooter(false)));
    //Command shoot = 
    c.addCommands(RapidShoot(3, false),b,pickUpBalls,RapidShoot(-1, false));//, pickUpBalls,RapidShoot());
    return c;
  }
  private RunCommand disableDevices() {
    return new RunCommand(()->{
      m_intake.powerIntake(0);
      m_intake.powerConveyor(0);
      m_shooter.powerShooter(false);
    });
  }

  
  private enum CommandsToChoose {
    Default, DirectTrench, Wall, DirectRndzvs, PickupMissedBalls, Test, MidZone, Shoot3, Shoot3Drive, SuperTrench;
  }

  private CommandsToChoose select() {
    return m_chooser.getSelected();
    //return CommandsToChoose.Test;
  }
  private Command TimedShoot(int setBalls, boolean aim, double timeout) {
    Command main = new RunCommand(()->{
      m_shooter.powerShooter(true);
      boolean shoot = m_shooter.atSpeed();
      boolean charge = !m_intake.ballInShooter();
      m_intake.powerConveyor(charge || shoot?1:0);
      m_intake.powerIntake(0);
      double chassisTurn = aim?0*m_shooter.AutoAimAndShoot():0;
      m_driveTrain.tankDriveVolts(chassisTurn, -chassisTurn);
    }, m_driveTrain, m_shooter, m_intake).withInterrupt(()->false)
    .withTimeout(timeout)
    .andThen(()->{m_shooter.enableLimelight(false); m_shooter.powerShooter(false); });
    return main;
  }
  private SequentialCommandGroup SafetyShoot(int setBalls, boolean aim, double timeout) {
    Command main = new RunCommand(()->{
      if (setBalls >= 0) {
        m_intake.setBallsInRobot(setBalls);
      }
      m_shooter.powerShooter(true);

      boolean shoot = m_shooter.atSpeed();
      boolean charge = !m_intake.ballInShooter();
      //if (shoot) System.out.println("Ready to shoot");
      //if (charge) System.out.println("Charging up");
      
      m_intake.powerConveyor(charge || shoot?1:0);
      m_intake.powerIntake(0);

      double chassisTurn = aim?0*m_shooter.AutoAimAndShoot():0;
      m_driveTrain.tankDriveVolts(chassisTurn, -chassisTurn);
    }, m_driveTrain, m_shooter, m_intake)
    .withInterrupt(()->{
      return m_shooter.shotAllBalls();
    }).withTimeout(timeout)
    .andThen(()->{m_shooter.enableLimelight(false); m_shooter.powerShooter(false); });
    return new InstantCommand(()->m_shooter.setBallCount(setBalls)).andThen(main);
  }
  private SequentialCommandGroup RapidShoot(int setBalls, boolean aim) {
    return new RunCommand(()->{
      if (setBalls >= 0) {
        m_intake.setBallsInRobot(setBalls);
      }
      m_shooter.powerShooter(true);

      boolean shoot = m_shooter.atSpeed();
      boolean charge = !m_intake.ballInShooter();
      if (shoot) System.out.println("Ready to shoot");
      if (charge) System.out.println("Charging up");
      m_intake.powerConveyor(charge || shoot?1:0);
      m_intake.powerIntake(0);

      double chassisTurn = aim?0*m_shooter.AutoAimAndShoot():0;
      m_driveTrain.tankDriveVolts(chassisTurn, -chassisTurn);
    }, m_driveTrain, m_shooter, m_intake)
    .withInterrupt(()->{
      boolean a = m_intake.getBallsInRobot() <= 0;
      boolean b = false && m_intake.ballInIntake();
      if (a) System.out.println("A!");
      if (b) System.out.println("B!");
      return a || b;
    }).withTimeout(7)
    .andThen(()->{m_shooter.enableLimelight(false); m_shooter.powerShooter(false); });
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
  /**
   * This is THE BEST (or BackwardsRamseteCommand) command to use for Ramsete control. As of 2/28/20
   * @param file The name of the file you are reading.
   * @return The Command (in type SequentialCommandGroup because why not), 
   * that resets odometry and stops after
   * @throws IOException just add a throws IOException in front of your method
   */
  private SequentialCommandGroup RamseteCommand(String file) throws IOException {
    Trajectory jsonTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(
        "/home/lvuser/deploy/output/"+file+".wpilib.json"));
    System.out.println("EasyRamseteCommand loaded \'"+file+"\' successfully.");
    return new SequentialCommandGroup(
      new InstantCommand(()->{
        m_driveTrain.setDirection(false);
        m_driveTrain.resetOdometry(jsonTrajectory.getInitialPose());
      }, m_driveTrain),
    new RamseteCommand(
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
    ),
    new InstantCommand(()->{
      m_driveTrain.tankDriveVolts(0, 0);
    })
    );
  }
  
  private SequentialCommandGroup BackwardsRamseteCommand(String file) throws IOException {
    Trajectory jsonTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(
        "/home/lvuser/deploy/output/"+file+".wpilib.json"));
    System.out.println("BackwardsRamseteCommand loaded \'"+file+"\' successfully.");
    return new SequentialCommandGroup(
      new InstantCommand(()->{
        m_driveTrain.setDirection(true);
        m_driveTrain.resetOdometry(jsonTrajectory.getInitialPose());
      }, m_driveTrain),
    new RamseteCommand(
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
        (l,r)->m_driveTrain.tankDriveVolts(-l,-r),
        //m_driveTrain::tankDriveVolts,
        m_driveTrain
    ),
    new InstantCommand(()->{
      m_driveTrain.tankDriveVolts(0, 0);
    })
    );
  }
}
