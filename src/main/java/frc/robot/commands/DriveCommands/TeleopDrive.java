package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.DriveConstants;

public class TeleopDrive extends Command {
  private final Joystick m_joystick;
  private final DriveTrain m_driveTrain;
  private double x, y, rot;

  /** 
   * Drives the robot during Teleop.
   * 
   * @param driveTrain The DriveTrain to be used.
   * @param joystick The joystick to read values from.
   */
  public TeleopDrive(Joystick joystick, DriveTrain driveTrain) {
    m_joystick = joystick;
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.coastAll();
    x = 0;
    y = 0;
    rot = 0;
    m_driveTrain.setOrientation(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = (Math.abs(m_joystick.getY()) < DriveConstants.translationalDeadband ? 0 : -m_joystick.getY());
    y = (Math.abs(m_joystick.getX()) < DriveConstants.translationalDeadband ? 0 : m_joystick.getX());
    rot = (Math.abs(m_joystick.getZ()) < DriveConstants.rotationalDeadband ? 0 : m_joystick.getZ());    
    m_driveTrain.teleopDrive(x, -y, -rot);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
