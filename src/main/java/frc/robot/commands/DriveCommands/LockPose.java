package frc.robot.commands.DriveCommands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class LockPose extends Command {
  private final DriveTrain m_driveTrain;
  private NeutralModeValue mode;

  /**
   * Sets the pose of the robot to be the locked pose: all modules' angles form an X.
   * 
   * @param driveTrain The DriveTrain to change its pose to the locked pose.
   */
  public LockPose(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.brakeAll();
    m_driveTrain.lockPose();
    mode = m_driveTrain.getIdleMode();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.lockPose();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setIdleMode(mode);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
