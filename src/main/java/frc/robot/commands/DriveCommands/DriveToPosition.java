package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Lib.AdvancedPose2D;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPosition extends Command {
  private final DriveTrain m_driveTrain;
  private final AdvancedPose2D m_desiredPose;
  /** Creates a new DriveToPosition. */
  public DriveToPosition(DriveTrain driveTrain, AdvancedPose2D desiredPose) {
    m_driveTrain = driveTrain;
    m_desiredPose = desiredPose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.setPIDSetpoints(m_desiredPose.getX(), m_desiredPose.getY(), m_desiredPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.PIDDrive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.atSetpoints();
  }
}
