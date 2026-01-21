package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final String m_name;

  private final TalonFX m_driveMotor, m_turningMotor;

  private final TalonFXConfiguration m_driveConfig, m_turnConfig;
  private final CANcoderConfiguration m_absEncoderConfig;

  private final CANcoder m_absoluteEncoder;

  private double absoluteEncoderOffset, turningFactor;
  private final boolean driveInverted, turnReversed, absReversed;
  
  private final SlewRateLimiter driveAccelLimiter;
  private final ProfiledPIDController turningController;

  private SwerveModuleState m_desiredState = new SwerveModuleState();

  private final GenericEntry desiredStateSender, wheelAngle, currentStateSender;

  /**
   * Constructs a SwerveModule.
   * 
   * @param name                   The name of the module.
   * @param driveMotorChannel      The CAN ID of the drive motor.
   * @param turningMotorChannel    The CAN ID of the turning motor.
   * @param turningEncoderChannel  The CAN ID of the turning encoder.
   * @param driveMotorReversed     Whether the drive encoder is reversed.
   * @param turningMotorReversed   Whether the turning encoder is reversed.
   * @param encoderOffset          The offset, in degrees, of the absolute encoder.
   * @param absoluteEncoderReversed  Whether the absolute encoder is reversed (negative).
   */
  public SwerveModule(String name, int driveMotorChannel, int turningMotorChannel, 
                      int absoluteEncoderID, boolean driveMotorReversed, boolean turningMotorReversed, 
                      double encoderOffset, boolean absoluteEncoderReversed) {
    /** Name */
    m_name = name;
    
    /** Motors */
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);

    m_driveConfig = new TalonFXConfiguration();
    m_turnConfig = new TalonFXConfiguration();
    m_absEncoderConfig = new CANcoderConfiguration();
    
    driveInverted = driveMotorReversed;
    turnReversed = turningMotorReversed;
    
    /** PIDController */
    turningController = new ProfiledPIDController(ModuleConstants.angleKP, 
                                                  ModuleConstants.angleKI, 
                                                  ModuleConstants.angleKD, 
                                                  ModuleConstants.angleControllerConstraints);
    turningController.setTolerance(ModuleConstants.angleKTolerance);
    turningController.enableContinuousInput(-180, 180);

    driveAccelLimiter = new SlewRateLimiter(2);

    /** Absolute Encoder */
    m_absoluteEncoder = new CANcoder(absoluteEncoderID);
    absoluteEncoderOffset = encoderOffset;
    absReversed = absoluteEncoderReversed;

    /** Configs */
    configAngleMotorDefault();
    configDriveMotorDefault();
    configAbsoluteEncoderDefault();

    resetEncoders();

    /** DashBoard Initialization */
    wheelAngle = IOConstants.DiagnosticTab.add(m_name + "'s angle", getAbsoluteEncoder()).getEntry();
    desiredStateSender = IOConstants.DiagnosticTab.add(m_name + "'s desired state", getState().toString()).getEntry();
    currentStateSender = IOConstants.DiagnosticTab.add(m_name + "'s current state", getState().toString()).getEntry();

  }

    /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isNeutral) {
    SwerveModuleState state = desiredState;
    state.optimize(getAngle());

    // m_driveMotor.setControl(new VelocityVoltage(state.speedMetersPerSecond).withSlot(0));
    final double driveOutput = state.speedMetersPerSecond / DriveConstants.maxSpeedMetersPerSecond;
    m_driveMotor.set(driveAccelLimiter.calculate(driveOutput));
    setAngle(state, isNeutral);

    SmartDashboard.putNumber("drive error", m_driveMotor.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber("drivingFactor", m_driveMotor.getClosedLoopOutput().getValueAsDouble());

    desiredStateSender.setString(desiredState.toString());
  }

  /**
   * Sets the angle of the module's turn motor.
   * 
   * @param desiredState The desired state of the module.
   */
  public void setAngle(SwerveModuleState desiredState, boolean isNeutral) {
    m_desiredState = desiredState;
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    // Rotation2d angle = 
    //   (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.maxSpeedMetersPerSecond * .01)) 
    //    ? getAngle() : desiredState.angle;

    turningController.setGoal(desiredState.angle.getDegrees());
    SmartDashboard.putNumber("turn error", turningController.getPositionError());
    SmartDashboard.putNumber("turningFactor", turningController.calculate(getAbsoluteEncoder()));
    m_turningMotor.set(isNeutral ? 0 : -turningController.calculate(getAbsoluteEncoder()));
    }

  /** 
   * Sets the state of the module, ONLY USE FOR LOCKED MODE.
   * 
   * @param lockedState The state with which to lock the module 
   */
  public void setLockedState(SwerveModuleState lockedState) {
    lockedState.optimize(getAngle());

    m_desiredState = lockedState;

    stop();
    m_turningMotor.set(turningController.atSetpoint() ? 0 : 
                       turningController.calculate(getAbsoluteEncoder(), lockedState.angle.getDegrees()));
  }

  /** @return The angle, in degrees, of the module. */
  public double getAbsoluteEncoder() {
    double angle = m_absoluteEncoder.getPosition().getValueAsDouble() * 360;
    angle -= absoluteEncoderOffset;
    angle = MathUtil.inputModulus(angle, -180, 180);
    return (absReversed ? -1 : 1) * angle;
  }


  /** Sets the default configuration of the angle motor. */
  private void configAngleMotorDefault() {
    m_turnConfig.Audio.BeepOnBoot = false;
    m_turnConfig.Audio.BeepOnConfig = false;

    m_turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    m_turnConfig.Feedback.SensorToMechanismRatio = 1;
    m_turnConfig.Feedback.RotorToSensorRatio = 1; //ModuleConstants.angleGearRatio
    // m_turnConfig.Feedback.FeedbackRemoteSensorID = m_absoluteEncoder.getDeviceID();

    m_turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    m_turnConfig.Slot0.kP = ModuleConstants.angleKP;
    m_turnConfig.Slot0.kI = ModuleConstants.angleKI;
    m_turnConfig.Slot0.kD = ModuleConstants.angleKD;

    m_turnConfig.Voltage.PeakForwardVoltage = ModuleConstants.maxVoltage;
    m_turnConfig.Voltage.PeakReverseVoltage = -ModuleConstants.maxVoltage;

    m_turnConfig.MotorOutput.NeutralMode = ModuleConstants.angleNeutralMode;
    m_turnConfig.MotorOutput.Inverted = (turnReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);

    configAngleMotor();
    Timer.delay(1);
  }

  /** Sets the default configuration of the drive motor. */
  private void configDriveMotorDefault() {
    m_driveConfig.Audio.BeepOnBoot = false;
    m_driveConfig.Audio.BeepOnConfig = false;

    m_driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    m_driveConfig.Feedback.SensorToMechanismRatio = ModuleConstants.driveMotorConversionFactor;

    m_driveConfig.ClosedLoopGeneral.ContinuousWrap = false;

    m_driveConfig.Slot0.kP = ModuleConstants.driveKP;
    m_driveConfig.Slot0.kI = ModuleConstants.driveKI;
    m_driveConfig.Slot0.kD = ModuleConstants.driveKD;

    m_driveConfig.Voltage.PeakForwardVoltage = ModuleConstants.maxVoltage;
    m_driveConfig.Voltage.PeakReverseVoltage = -ModuleConstants.maxVoltage;

    m_driveConfig.MotorOutput.NeutralMode = ModuleConstants.initialDriveNeutralMode;
    m_driveConfig.MotorOutput.Inverted = (driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);

    Timer.delay(1);
    configDriveMotor();
  }

  /** Sets the default configuration of the absolute encoder. */
  private void configAbsoluteEncoderDefault() {
    m_absEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ModuleConstants.absoluteEncoderRange;
    m_absEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    Timer.delay(1);
    configAbsoluteEncoder();
  }

  /** Applies the configuration of the turning motor to m_turnConfig */
  public void configAngleMotor() {
    m_turningMotor.getConfigurator().apply(m_turnConfig);
  }

  /** Applies the configuration of the drive motor to m_driveConfig */
  public void configDriveMotor() {
    m_driveMotor.getConfigurator().apply(m_driveConfig);
  }

  /** Applies the configuration of the absolute encoder to m_absEncoderConfig */
  public void configAbsoluteEncoder() {
    m_absoluteEncoder.getConfigurator().apply(m_absEncoderConfig);
  }

  /** Resets the {@link SwerveModule}'s drive encoder. */
  public void resetEncoders() {
    m_driveMotor.setPosition(0);
  }

  /** Sets the module's drive motor's idle mode to brake. */
  public void brake() {
    m_driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configDriveMotor();
  }

  /** Sets the module's drive motor's idle mode to coast. */
  public void coast() {
    m_driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configDriveMotor();
  }

  /** 
   * Sets the module's drive motor's NeutralMode.
   * 
   * @param mode The NeutralMode to set the module's drive motor to.
   */
  public void setNeutralMode(NeutralModeValue mode) {
    m_driveConfig.MotorOutput.NeutralMode = mode;
    configDriveMotor();
  }

  /** @return The current NeutralMode of the Module. */
  public NeutralModeValue getNeutralMode() {
    return m_driveConfig.MotorOutput.NeutralMode;
  }
  
  /** @return The current {@link SwerveModuleState} of the module. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getVelocity().getValueAsDouble(), getAngle());
  }

  /** @return The current position of the module - essentially a {@link SwerveModuleState}, 
   *                                               but using distance traveled rather than velocity. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveMotor.getPosition().getValueAsDouble(), 
                                    new Rotation2d(Units.degreesToRadians(getAbsoluteEncoder())));
  }

  /** @return The current rotation position of the module. */
  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(getAbsoluteEncoder());
  }

  /** @return The drive motor of the module. */
  public TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  /** @return The turn motor of the module. */
  public TalonFX getTurnMotor() {
    return m_turningMotor;
  }

  /** Stops the module's drive motor from moving. */
  public void stop() {
    m_driveMotor.stopMotor();
  }

  /** Stops the module's angle motor from moving. */
  public void stopTurn() {
    m_turningMotor.stopMotor();
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /** @return The distance the drive motor has moved. */
  public double getDistance() {
    return m_driveMotor.getPosition().getValueAsDouble() * ModuleConstants.driveMotorConversionFactor;
  }

  /** Posts module values to ShuffleBoard. */
  public void update() {
    wheelAngle.setDouble(getAbsoluteEncoder());
    currentStateSender.setString(getState().toString());
    Double[] array = {turningController.getPositionError(), turningController.getSetpoint().position, turningFactor};
  }
}