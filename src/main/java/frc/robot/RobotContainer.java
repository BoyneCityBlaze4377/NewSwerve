package frc.robot;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.Auton.NoAuton;
import frc.robot.commands.DriveCommands.ForceRobotOrientation;
import frc.robot.commands.DriveCommands.LockPose;
import frc.robot.commands.DriveCommands.QuickBrake;
import frc.robot.commands.DriveCommands.SlowMode;
import frc.robot.commands.DriveCommands.SwitchOrientation;
import frc.robot.commands.DriveCommands.TeleopDrive;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** JOYSTICKS */
  private final Joystick m_driverStick = new Joystick(IOConstants.driverControllerID);
  
  /** SUBSYSTEMS */
  private final DriveTrain m_driveTrain = new DriveTrain();

  /** Auton Chooser */
  private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  /* COMMANDS */
  // DriveTrain
  //Main Commands
  private final Command TeleopDrive = new TeleopDrive(m_driverStick, m_driveTrain);
  private final Command LockPose = new LockPose(m_driveTrain); 
  // private final Command SwitchBrake = new SwitchBrake(m_driveTrain);
  private final Command SwitchOrientation = new SwitchOrientation(m_driveTrain);
  private final Command QuickBrake = new QuickBrake(m_driveTrain);
  private final Command SlowMode = new SlowMode(m_driveTrain);
  private final Command RobotOrient = new ForceRobotOrientation(m_driveTrain);

  /** AUTONS */
  private final NoAuton NoAuton = new NoAuton();
  //Testing

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveTrain.setDefaultCommand(TeleopDrive);
    // m_driveTrain.setDefaultCommand(tTESTDRIVECommand);
    configureButtonBindings();

    configAutonChooser();
    IOConstants.ConfigTab.add("Auton Chooser", autonChooser)
                         .withWidget("ComboBox Chooser")
                         .withProperties(Map.of("sort_options", true));
  }

  public void setAlliance(Optional<Alliance> alliance) {
    m_driveTrain.setAlliance(alliance.isPresent() ? alliance.get() : Alliance.Blue);
  }

  public void setDriveOrientation(boolean fieldOriented) {
    m_driveTrain.setOrientation(fieldOriented);
  }

  public void configAutonChooser() {
    autonChooser.setDefaultOption("No Auton", NoAuton);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /* DRIVER */
    //Main Commands
    new JoystickButton(m_driverStick, IOConstants.quickBrakeButtonID).whileTrue(QuickBrake);
    new JoystickButton(m_driverStick, IOConstants.slowModeButtonID).whileTrue(SlowMode);
    new JoystickButton(m_driverStick, IOConstants.lockPoseButtonID).whileTrue(LockPose);
    new JoystickButton(m_driverStick, IOConstants.switchOrientationButtonID).onTrue(SwitchOrientation);
    new JoystickButton(m_driverStick, IOConstants.robotOrientButtonID).whileTrue(RobotOrient);
    //Testing
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return NoAuton;
  }
}