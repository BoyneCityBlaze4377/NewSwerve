package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IOConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final GenericEntry fmsInfo, voltage;
  private static final Timer robotTimer = new Timer();
  private static Alliance m_alliance = Alliance.Blue;
  //Remy Test


  public Robot() {
    m_robotContainer = new RobotContainer();
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    fmsInfo = IOConstants.ConfigTab.add("FMSInfo", 0)
                                   .withWidget("FMSInfo")
                                   .getEntry();
    voltage = IOConstants.TeleopTab.add("Battery Voltage", RobotController.getBatteryVoltage())
                                   .withWidget("Voltage View")
                                   .getEntry();
  }
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Shuffleboard.selectTab(IOConstants.ConfigTab.getTitle());
    robotTimer.reset();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    voltage.setDouble(RobotController.getBatteryVoltage());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setAlliance(DriverStation.getAlliance());
    robotTimer.stop();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.setAlliance(DriverStation.getAlliance());
    if (DriverStation.getAlliance().isPresent()) m_alliance = DriverStation.getAlliance().get();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.setAlliance(DriverStation.getAlliance());
    
    Shuffleboard.selectTab(IOConstants.AutonTab.getTitle());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    robotTimer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Shuffleboard.selectTab(IOConstants.TeleopTab.getTitle());

    m_robotContainer.setAlliance(DriverStation.getAlliance());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    robotTimer.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public static double getRobotTime() {
    return robotTimer.get();
  }

  public static Alliance getRobotAlliance() {
    return m_alliance;
  }
}
