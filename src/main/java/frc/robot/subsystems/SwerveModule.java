package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;

public class SwerveModule {
  public int moduleNumber;
  private SwerveModuleConstants moduleConstants;

  private TalonFX angleMotor;
  private TalonFX driveMotor;

  private VelocityVoltage driveMotorControl;
  private PositionVoltage angleMotorControl;

  private CANcoder angleEncoder;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.moduleConstants = moduleConstants;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new TalonFX(moduleConstants.angleMotorID);
    configAngleMotor();
    angleMotorControl = new PositionVoltage(angleMotor.getPosition().getValueAsDouble());

    /* Drive Motor Config */
    driveMotor = new TalonFX(moduleConstants.driveMotorID);
    configDriveMotor();
    driveMotorControl = new VelocityVoltage(0);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void configAngleEncoder() {
    angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
    CANcoderConfiguration angleEncoderConfiguration = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
      .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
      .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
      // This will make the angleEncoder.getAbsolutePosition() method return the value with offset
      // The offsets must be removed for calibration.
      .withMagnetOffset(moduleConstants.angleOffset.getRotations())
    );
    angleEncoder.getConfigurator().apply(angleEncoderConfiguration);
  }

  private void configAngleMotor() {
    angleMotor.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration()
    .withMotionMagic(new MotionMagicConfigs()

    ).withCurrentLimits(new CurrentLimitsConfigs()
      .withStatorCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
    ).withMotorOutput(new MotorOutputConfigs()
      .withInverted(Constants.Swerve.angleInvert ? InvertedValue.CounterClockwise_Positive : InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(Constants.Swerve.angleNeutralMode)
    ).withSlot0(new Slot0Configs()
      .withKP(Constants.Swerve.angleKP)
      .withKI(Constants.Swerve.angleKI)
      .withKD(Constants.Swerve.angleKD)
    ).withFeedback(new FeedbackConfigs()
      // Using FusedCANcoder, which uses the integrated encoder after getting an absolute angle from CANcoder
      .withSensorToMechanismRatio(1)
      .withRotorToSensorRatio(Constants.Swerve.angleConversionFactor)
      .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
      .withFeedbackRemoteSensorID(moduleConstants.cancoderID)
    );
    angleMotor.getConfigurator().apply(angleMotorConfig);
  }

  private void configDriveMotor() {
    driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration()
    .withMotionMagic(new MotionMagicConfigs()

    ).withCurrentLimits(new CurrentLimitsConfigs()
      .withStatorCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit)
    ).withMotorOutput(new MotorOutputConfigs()
      .withInverted(Constants.Swerve.driveInvert ? InvertedValue.CounterClockwise_Positive : InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(Constants.Swerve.driveNeutralMode)
    ).withSlot0(new Slot0Configs()
      .withKP(Constants.Swerve.driveKP)
      .withKI(Constants.Swerve.driveKI)
      .withKD(Constants.Swerve.driveKD)
      .withKS(Constants.Swerve.driveKS)
      .withKV(Constants.Swerve.driveKV)
      .withKA(Constants.Swerve.driveKA)
    ).withFeedback(new FeedbackConfigs()
      .withSensorToMechanismRatio(Constants.Swerve.driveGearRatio)
      .withRotorToSensorRatio(1)
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
    );
    driveMotor.getConfigurator().apply(driveMotorConfig);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveMotor.setControl(driveMotorControl.withVelocity(desiredState.speedMetersPerSecond / Constants.Swerve.wheelCircumference));
    }
  }

  public void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    if (Math.abs(desiredState.speedMetersPerSecond) > (Constants.Swerve.maxSpeed * 0.01)) {
      angleMotor.setControl(angleMotorControl.withPosition(desiredState.angle.getRotations()));
    }
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
  }

  public SwerveModulePosition getPosition() {
    double wheelPosition = driveMotor.getPosition().getValueAsDouble() * Constants.Swerve.wheelCircumference;
    return new SwerveModulePosition(wheelPosition, getAngle());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
  }

  public SwerveModuleState getState() {
    double wheelVelocity = driveMotor.getVelocity().getValueAsDouble() * Constants.Swerve.wheelCircumference;
    return new SwerveModuleState(wheelVelocity, getAngle());
  }
}
