package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
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

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final String m_name;

  private final TalonFX m_driveMotor, m_turningMotor;

  private final TalonFXConfiguration m_driveConfig, m_turnConfig;
  private final CANcoderConfiguration m_absEncoderConfig;

  private final CANcoder m_absoluteEncoder;

  private double AnalogEncoderOffset, turningFactor;
  private final boolean driveInverted, turnReversed, absReversed;

  private Rotation2d lastAngle;
  
  private final SlewRateLimiter filter;
  private final ProfiledPIDController turningController;

  private SwerveModuleState m_desiredState;

  private final GenericEntry //desiredStateSender, 
  TESTwheelAngle;

  /**
   * Constructs a SwerveModule.
   * 
   * @param name                   The name of the module.
   * @param driveMotorChannel      The channel of the drive motor.
   * @param turningMotorChannel    The channel of the turning motor.
   * @param turningEncoderChannel  The channels of the turning encoder.
   * @param driveMotorReversed     Whether the drive encoder is reversed.
   * @param turningMotorReversed   Whether the turning encoder is reversed.
   * @param encoderOffset          The offset, in degrees, of the absolute encoder.
   * @param absoluteEncoderReversed  Whether the absolute encoder is reversed (negative).
   */
  public SwerveModule(String name, int driveMotorChannel, int turningMotorChannel, 
                      int turningEncoderChannel, boolean driveMotorReversed, boolean turningMotorReversed, 
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
    
    //configAngleMotorDefault();
    //configDriveMotorDefault();
    configAbsoluteEncoderDefault();
    
    /** PIDController */
    turningController = new ProfiledPIDController(ModuleConstants.angleKP, 
                                                  ModuleConstants.angleKI, 
                                                  ModuleConstants.angleKD, 
                                                  ModuleConstants.angleControllerConstraints);
    turningController.setTolerance(ModuleConstants.kTolerance);
    turningController.enableContinuousInput(-180, 180);

    filter = new SlewRateLimiter(2);

    /** Absolute Encoder */
    m_absoluteEncoder = new CANcoder(turningEncoderChannel);
    AnalogEncoderOffset = encoderOffset;
    absReversed = absoluteEncoderReversed;

    //resetEncoders();

    /** DashBoard Initialization */
    TESTwheelAngle = IOConstants.DiagnosticTab.add(m_name + "'s angle TEST", getAbsoluteEncoder()).getEntry();
    // desiredStateSender = IOConstants.DiagnosticTab.add(m_name + "'s desired state", getState().toString()).getEntry();

    // LastAngle
    lastAngle = getState().angle;
  }

  /** @return The angle, in degrees, of the module */
  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(getAbsoluteEncoder());
  }

  /** @return The current state of the module. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getVelocity().getValueAsDouble(), getAngle());
  }

  /** @return The current position of the module. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveMotor.getPosition().getValueAsDouble(), 
                                    new Rotation2d(Units.degreesToRadians(getAbsoluteEncoder())));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isNeutral) {
    SwerveModuleState state = desiredState;
    state.optimize(lastAngle);

    final double driveOutput = state.speedMetersPerSecond / DriveConstants.maxSpeedMetersPerSecond;

    m_driveMotor.set(filter.calculate(driveOutput));
    setAngle(state, isNeutral);

    //desiredStateSender.setString(desiredState.toString());
  }

  /**
   * Sets the angle of the module's turn motor.
   * 
   * @param desiredState The desired state of the module.
   */
  public void setAngle(SwerveModuleState desiredState, boolean isNeutral) {
    m_desiredState = desiredState;
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = 
        (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.maxSpeedMetersPerSecond * .01)) ? lastAngle : desiredState.angle;
    
    turningFactor = MathUtil.clamp(turningController.calculate(getAbsoluteEncoder(), angle.getDegrees()), 
                                   -ModuleConstants.kMaxOutput, ModuleConstants.kMaxOutput);
    
    m_turningMotor.set(isNeutral || turningController.atSetpoint() ? 0 : -turningFactor);
    lastAngle = angle;
  }

  /** Resets all of the SwerveModule's encoders. */
  public void resetEncoders() {
    m_driveMotor.setPosition(0);
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

  /** @return The angle, in degrees, of the module. */
  public double getAbsoluteEncoder() {
    double angle = m_absoluteEncoder.getPosition().getValueAsDouble() * 360;
    angle -= AnalogEncoderOffset;
    angle = MathUtil.inputModulus(angle, -180, 180);
    return (absReversed ? -1 : 1) * angle;
  }

  /** Posts module values to ShuffleBoard. */
  public void update() {
    TESTwheelAngle.setDouble(getAbsoluteEncoder());
    //System.out.println(getAbsoluteEncoder());
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /** @return The distance the drive motor has moved. */
  public double getDistance() {
    return m_driveMotor.getPosition().getValueAsDouble();
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

  /** Sets the configuration of the turning motor to m_turnConfig */
  public void configAngleMotor() {
    m_turningMotor.getConfigurator().apply(m_turnConfig);
  }

  /** Sets the configuration of the drive motor to m_driveConfig */
  public void configDriveMotor() {
    m_driveMotor.getConfigurator().apply(m_driveConfig);
  }

  /** Sets the configuration of the absolute encoder to m_absEncoderConfig */
  public void configAbsoluteEncoder() {
    m_absoluteEncoder.getConfigurator().apply(m_absEncoderConfig);
  }

  /** Sets the default configuration of the angle motor. */
  private void configAngleMotorDefault() {
    m_turnConfig.MotorOutput.NeutralMode = ModuleConstants.angleNeutralMode;
    m_turnConfig.MotorOutput.Inverted = (turnReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);

    configAngleMotor();
    Timer.delay(1);
  }

  /** Sets the default configuration of the drive motor. */
  private void configDriveMotorDefault() {
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder resolution.
    
    // m_driveConfig.encoder.velocityConversionFactor(ModuleConstants.driveMotorConversionFactor/60);
    // m_driveConfig.encoder.positionConversionFactor(ModuleConstants.driveMotorConversionFactor);

    m_driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_driveConfig.MotorOutput.Inverted = (driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
    m_driveConfig.ClosedLoopGeneral.ContinuousWrap = false;

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

  /** 
   * Sets the state of the module, ONLY USE FOR LOCKED MODE.
   * 
   * @param lockedState The state with which to lock the module 
   */
  public void setLockedState(SwerveModuleState lockedState) {
    lockedState.optimize(lastAngle);

    m_driveMotor.set(0);
    turningFactor = turningController.calculate(getAbsoluteEncoder(), lockedState.angle.getDegrees());

    turningFactor = MathUtil.clamp(turningFactor, -ModuleConstants.kMaxOutput, ModuleConstants.kMaxOutput);

    m_turningMotor.set(turningController.atSetpoint() ? 0 : -turningFactor);
    lastAngle = lockedState.angle;
  }

  /** @return The current NeutralMode of the Module. */
  public NeutralModeValue getNeutralMode() {
    return m_driveConfig.MotorOutput.NeutralMode;
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
}