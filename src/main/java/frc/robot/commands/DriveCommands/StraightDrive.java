package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StraightDrive extends Command {
  private final DriveTrain m_driveTrain;
  private final Joystick m_joystick;
  private boolean orientation;
  /** Creates a new StraightDrive. */
  public StraightDrive(DriveTrain driveTrain, Joystick joystick) {
    m_driveTrain = driveTrain;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    orientation = m_driveTrain.isFieldOriented();
    m_driveTrain.setOrientation(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.straightDrive(MathUtil.applyDeadband(-m_joystick.getY(), DriveConstants.translationalDeadband));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setOrientation(orientation);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
