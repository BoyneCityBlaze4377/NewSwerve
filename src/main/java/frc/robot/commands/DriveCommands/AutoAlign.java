// package frc.robot.commands.DriveCommands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.AutoAimConstants;
// import frc.robot.Constants.AutoAimConstants.Alignment;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.AutoAimSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class AutoAlign extends Command {
//   private final DriveTrain m_driveTrain;
//   private double xSpeed, ySpeed, rot, initialTargDis, givenTargDis, targetDistance, targetOffsetDeg, targetAngle, maxHorizOutput, 
//                   maxDisOutput, maxRotOutput;
//   private final PIDController angleController, horizController, distanceController;
//   private final Alignment m_alignment;
//   private final AutoAimSubsystem m_autoAimSubsystem;
//   private boolean orientation;

//   /** Creates a new LimeLightDrive. */
//   public AutoAlign(DriveTrain driveTrain, AutoAimSubsystem visionSubsystem, 
//                    double TargetDistance, Alignment alignment) {
//     m_driveTrain = driveTrain;
//     m_autoAimSubsystem = visionSubsystem;

//     givenTargDis = TargetDistance;
//     // switch (pov) {
//     //   case 270:
//     //     alignment = Alignment.left;
//     //   break;
//     //   case 90:
//     //     alignment = Alignment.right;
//     //   break;
//     //   default:
//     //     alignment = Alignment.center;
//     //   break;
//     // }
//     m_alignment = alignment;

//     horizController = new PIDController(AutoAimConstants.horizkP, 
//                                         AutoAimConstants.horizkI, 
//                                         AutoAimConstants.horizkD);
//     distanceController = new PIDController(AutoAimConstants.diskP, 
//                                            AutoAimConstants.diskI, 
//                                            AutoAimConstants.diskP);
//     angleController = new PIDController(AutoAimConstants.turnkP, 
//                                         AutoAimConstants.turnkI, 
//                                         AutoAimConstants.turnkD);

//     horizController.setTolerance(AutoAimConstants.horizkTolerance);
//     distanceController.setTolerance(AutoAimConstants.diskTolerance);
//     angleController.setTolerance(AutoAimConstants.turnkTolerance);

//     addRequirements(m_driveTrain);

//     maxHorizOutput = 3;
//     maxDisOutput = 3;
//     maxRotOutput = Math.PI * 3/2;

//     orientation = m_driveTrain.isFieldOriented();

//     initialTargDis = Units.inchesToMeters(10);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     targetOffsetDeg = (AutoAimConstants.offsetFromAlignment.get(m_alignment) == 0 ? 0 : 
//                       1 / Math.atan(AutoAimConstants.offsetFromAlignment.get(m_alignment) 
//                       / initialTargDis)) + AutoAimConstants.LLDefaultOffsetDegrees;
//     // targetAngle = m_driveTrain.getEstimatedStation();
//     // AutoAimConstants.angleFromReefStation.get(AutoAimConstants.reefStationFromAprilTagID.get(
//     //                                                         m_autoAimSubsystem.getTargetID()));
//     m_driveTrain.setOrientation(false);
//     targetAngle = 0;

//     SmartDashboard.putNumber("OFFSET", targetOffsetDeg);

//     targetDistance = initialTargDis;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     ySpeed = MathUtil.clamp(horizController.calculate(m_autoAimSubsystem.getTX(), targetOffsetDeg), 
//                             -maxHorizOutput, maxHorizOutput);
//     // xSpeed = MathUtil.clamp(distanceController.calculate(m_autoAimSubsystem.getDistanceMeasurementmm(), targetDistance * 1000), 
//     //                         -maxDisOutput, maxDisOutput);
//     rot = MathUtil.clamp(angleController.calculate(m_driveTrain.getHeading(), targetAngle), 
//                          -maxRotOutput, maxRotOutput);

//     if (horizController.atSetpoint()) targetDistance = givenTargDis;

//     SmartDashboard.putNumber("YSpeed", ySpeed);
//     SmartDashboard.putString("ALIGNMENT", m_alignment.toString());

//     m_driveTrain.autonDrive(-0, -ySpeed, -0);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_driveTrain.stop();
//     m_driveTrain.lockPose();
//     xSpeed = 0;
//     ySpeed = 0;
//     rot = 0;
//     m_driveTrain.setOrientation(orientation);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return (horizController.atSetpoint() && 
//             distanceController.atSetpoint()
//             && angleController.atSetpoint())
//             // || m_autoAimSubsystem.getTargetID() == 0
//             // || m_autoAimSubsystem.getDistanceMeasurementmm() == -1;
//             ;
//   }
// }
