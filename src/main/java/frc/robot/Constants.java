package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public class IOConstants {
    /* DASHBOARD TABS */
    public static final ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");
    public static final ShuffleboardTab AutonTab = Shuffleboard.getTab("Auton");
    public static final ShuffleboardTab DiagnosticTab = Shuffleboard.getTab("Diagnostic");
    public static final ShuffleboardTab ConfigTab = Shuffleboard.getTab("Configuration");

    /* CONTROLLER IDS */
    public static final int driverControllerID = 0;

    /* BUTTON IDS */
    /* Driver */
    //Main Functions
    public static final int quickBrakeButtonID = 6; //6
    public static final int slowModeButtonID = 3; //3
    public static final int switchOrientationButtonID = 4; //4
    public static final int lockPoseButtonID = 5; //5
    public static final int straightDriveButtonID = 2; //2
    public static final int autoDriveButtonID = 1; //1
    public static final int robotOrientButtonID = 9; //9
  }

  /* Physical constants of the DriveTrain */
  public static final class SwerveConstants {
    // Distance between centers of right and left wheels on robot in meters
    public static final double trackWidth = Units.inchesToMeters(36); //.9144
    
    // Distance between front and back wheels on robot in meters
    public static final double wheelBase = Units.inchesToMeters(36); //.9144
    
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(new Translation2d(wheelBase / 2, trackWidth / 2),
                                  new Translation2d(wheelBase / 2, -trackWidth / 2),
                                  new Translation2d(-wheelBase / 2, trackWidth / 2),
                                  new Translation2d(-wheelBase / 2, -trackWidth / 2));
  }

  /* Constants related to how the DriveTrain moves */
  public static final class DriveConstants {
    public static final double speedScaler = 1;

    public static final double maxSpeedMetersPerSecond = 4.25; //4.5 true max
    public static final double maxAccelerationMetersPerSecondSquared = 4;
    public static final double maxRotationSpeedRadiansPerSecond = Math.PI * .8;
    public static final double maxRotationAccelerationRadiansPerSecondSquared = Math.PI;

    public static final double translationalDeadband = .1;
    public static final double rotationalDeadband = .4;

    public static final double jerkCrashTheshold = maxAccelerationMetersPerSecondSquared * Math.sqrt(2);
    public static final double crashDebounceTime = .1;

    public static final double ksVolts = 5;
    public static final double kvVoltSecondsPerMeter = 4;
    public static final double kaVoltSecondsSquaredPerMeter = 1;

    public static final boolean gyroReversed = false;
  }

  /* Constants to create and configure SwerveModules */
  public static final class ModuleConstants {
    public static final double moduleDriveSlewRate = 2;

    public static final int frontLeftDriveMotorPort = 1;
    public static final int frontRightDriveMotorPort = 3;
    public static final int backLeftDriveMotorPort = 7;
    public static final int backRightDriveMotorPort = 5;

    public static final int frontLeftTurningMotorPort = 2;
    public static final int frontRightTurningMotorPort = 4;
    public static final int backLeftTurningMotorPort = 8;
    public static final int backRightTurningMotorPort = 6;
    
    public static final int frontLeftTurningEncoderPort = 0;
    public static final int frontRightTurningEncoderPort = 1;
    public static final int backLeftTurningEncoderPort = 3;
    public static final int backRightTurningEncoderPort = 2;

    public static final boolean frontLeftTurningMotorReversed = false;    
    public static final boolean frontRightTurningMotorReversed = false;
    public static final boolean backLeftTurningMotorReversed = false;
    public static final boolean backRightTurningMotorReversed = false;

    public static final boolean frontLeftDriveMotorReversed = false;    
    public static final boolean frontRightDriveMotorReversed = false;
    public static final boolean backLeftDriveMotorReversed = false;
    public static final boolean backRightDriveMotorReversed = false;
    
    public static final boolean frontLeftAbsReversed = false;    
    public static final boolean frontRightAbsReversed = false;
    public static final boolean backLeftAbsReversed = false;
    public static final boolean backRightAbsReversed = false;

    public static final double frontLeftAnalogEncoderOffset = 4.69;  
    public static final double frontRightAnalogEncoderOffset = 75.88;
    public static final double backLeftAnalogEncoderOffset = 162.39;
    public static final double backRightAnalogEncoderOffset = 63.79;

    public static final double maxModuleSpeedMetersPerSecond = 4.5;
    public static final double maxModuleAccelerationMetersPerSecondSquared = 4;
    public static final double maxModuleAngularSpeedDegreesPerSecond = 360;
    public static final double maxModuleAngularAccelerationDegreesPerSecondSquared = 360;

    public static final double encoderCPR = 1;
    public static final double wheelDiameterMeters = Units.inchesToMeters(3.8125);
    public static final double driveGearRatio = 1 / 6.75;
    public static final double driveMotorConversionFactor = 
      // Assumes the encoders are directly mounted on the wheel shafts
      (wheelDiameterMeters * Math.PI) / (double) encoderCPR * driveGearRatio;

    public static final double angleGearRatio = (150/7);
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    public static final double voltageComp = 12.0;
    public static final int angleContinuousCurrentLimit = 20;
  
    public static final Constraints angleControllerConstraints = new Constraints(Math.PI * 2, Math.PI);
    public static final double angleKP = 0.01; //.01
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;
    public static final double kTolerance = .5;

    public static final double kMaxOutput = 0.95;
  
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
  }

  public class FieldConstants {
    public static final double fieldLength = 17.548;
    public static final double fieldWidth = 8.052;
  }

  public class AutoAimConstants {
    public static final double transkP = 2; //1.5
    public static final double transkI = 0; //.013
    public static final double transkD = 0; //0
    public static final double transkTolerance = .025;

    public static final double turnkP = 4.5; //4.5
    public static final double turnkI = .355; //.355
    public static final double turnkD = 0; //0
    public static final double turnkTolerance = .03;

    public static final Vector<N3> poseEstimateOdometryStdDev = VecBuilder.fill(.05, .05, Units.degreesToRadians(.2));
    public static final Vector<N3> poseEstimateVisionStdDev = VecBuilder.fill(.1, .1, Units.degreesToRadians(3));
    public static final Vector<N3> poseEstimateCrashVisionStdDev = VecBuilder.fill(.02, .02, Units.degreesToRadians(2));
  }

  public class AutonConstants {}

  public class SensorConstants {
    /** LIMELIGHT */
    public static final String limeLightName = "limelight";
  }

  public class RemyConstants {
    public static RobotConfig config;
    
  }
}