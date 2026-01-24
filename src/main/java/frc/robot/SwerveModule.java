package frc.robot;

import static frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;

  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CANcoder angleEncoder;

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);

  /* angle motor control requests */
  private final PositionVoltage anglePositionRequest = new PositionVoltage(0);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset();

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID());
    angleEncoder.getConfigurator().apply(getEncoderConfig());

    /* Angle Motor Config */
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID());
    mAngleMotor.getConfigurator().apply(getTurnMotorConfig());
    resetToAbsolute();

    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID());
    mDriveMotor.getConfigurator().apply(getDriveMotorConfig());
    mDriveMotor.getConfigurator().setPosition(0.0);

    angleEncoder.getAbsolutePosition().setUpdateFrequency(100);
    angleEncoder.optimizeBusUtilization();

    mDriveMotor.getVelocity().setUpdateFrequency(100);
    mDriveMotor.getPosition().setUpdateFrequency(100);
    mDriveMotor.getMotorVoltage().setUpdateFrequency(100);
    mDriveMotor.optimizeBusUtilization();
    mAngleMotor.getVelocity().setUpdateFrequency(100);
    mAngleMotor.getPosition().setUpdateFrequency(100);
    mAngleMotor.optimizeBusUtilization();
  }

  private TalonFXConfiguration getDriveMotorConfig() {
    var config = new TalonFXConfiguration();
    /* Motor Inverts and Neutral Mode */
    config.MotorOutput.Inverted = SwerveConstants.driveMotorInvert;
    config.MotorOutput.NeutralMode = SwerveConstants.driveNeutralMode;

    /* Gear Ratio Config */
    // swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.driveGearRatio;

    /* Current Limiting */
    config.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.driveEnableSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.driveSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLowerLimit = SwerveConstants.driveSupplyCurrentLower;
    config.CurrentLimits.SupplyCurrentLowerTime = SwerveConstants.driveSupplyCurrentLowerTime;

    config.CurrentLimits.StatorCurrentLimit = SwerveConstants.driveStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = SwerveConstants.driveEnableStatorCurrentLimit;

    /* PID Config */
    config.Slot0.kP = SwerveConstants.driveKP;
    config.Slot0.kI = SwerveConstants.driveKI;
    config.Slot0.kD = SwerveConstants.driveKD;

    config.Slot0.kS = SwerveConstants.driveKS;
    // VS/m * m/r_w / (r_m/r_w) = VS/r_m
    config.Slot0.kA =
        SwerveConstants.driveKA
            * Conversions.rotationsToMeters(1, SwerveConstants.wheelCircumference)
            / SwerveConstants.driveGearRatio;
    config.Slot0.kV =
        SwerveConstants.driveKV
            * Conversions.rotationsToMeters(1, SwerveConstants.wheelCircumference)
            / SwerveConstants.driveGearRatio;

    /* Open and Closed Loop Ramping */
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;

    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
    return config;
  }

  private TalonFXConfiguration getTurnMotorConfig() {
    var config = new TalonFXConfiguration();
    /* Motor Inverts and Neutral Mode */
    config.MotorOutput.Inverted = SwerveConstants.angleMotorInvert;
    config.MotorOutput.NeutralMode = SwerveConstants.angleNeutralMode;

    /* Gear Ratio and Wrapping Config */
    config.Feedback.SensorToMechanismRatio = SwerveConstants.angleGearRatio;
    config.ClosedLoopGeneral.ContinuousWrap = true;

    /* Current Limiting */
    config.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.angleEnableSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.angleSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLowerLimit = SwerveConstants.angleSupplyCurrentLower;
    config.CurrentLimits.SupplyCurrentLowerTime = SwerveConstants.angleSupplyCurrentLowerTime;

    config.CurrentLimits.StatorCurrentLimit = SwerveConstants.angleStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = SwerveConstants.angleEnableStatorCurrentLimit;

    /* PID Config */
    config.Slot0.kP = SwerveConstants.angleKP;
    config.Slot0.kI = SwerveConstants.angleKI;
    config.Slot0.kD = SwerveConstants.angleKD;

    return config;
  }

  private CANcoderConfiguration getEncoderConfig() {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SwerveConstants.cancoderInvert;
    return config;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState.optimize(getState().angle);

    // Cosine compensation
    var angleError = desiredState.angle.minus(getAngle());
    // Clamp scale factor to (0, 1) to prevent reversing
    // This shouldn't ever happen but just in case
    var velocityInDesiredDirection =
        MathUtil.clamp(angleError.getCos(), 0, 1) * desiredState.speedMetersPerSecond;

    setAngle(desiredState.angle);
    setSpeed(velocityInDesiredDirection, isOpenLoop);
  }

  private void setAngle(Rotation2d angle) {
    mAngleMotor.setControl(anglePositionRequest.withPosition(angle.getRotations()));
  }

  private void setSpeed(double speedMetersPerSecond, boolean isOpenLoop) {
    // Convert linear speed of wheel to motor speed
    var requestedVelocityRPS = wheelMeterToMotorRot(speedMetersPerSecond);

    double compensationVelocity =
        calculateCompensationVelocity(
            speedMetersPerSecond,
            mAngleMotor.getVelocity().getValueAsDouble(),
            SwerveConstants.azimuthCouplingRatio);
    var outputVelocity = requestedVelocityRPS + compensationVelocity;

    if (isOpenLoop) {
      driveDutyCycleRequest.Output =
          outputVelocity / wheelMeterToMotorRot(SwerveConstants.maxSpeed);
      mDriveMotor.setControl(driveDutyCycleRequest);
    } else {
      driveVelocityRequest.Velocity = outputVelocity;
      mDriveMotor.setControl(driveVelocityRequest);
    }
  }

  private double calculateCompensationVelocity(
      double desiredSpeedMetersPerSecond, double currentAngleVelocity, double couplingRatio) {
    // Calculate motor velocity required to hold wheel still at the current azimuth velocity
    // Don't compensate if requested velocity is 0 - just stop the motor
    if (desiredSpeedMetersPerSecond == 0) {
      return 0;
    } else {
      return currentAngleVelocity * couplingRatio;
    }
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
  }

  public void resetToAbsolute() {
    double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    mAngleMotor.setPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        motorRotToWheelMeter(mDriveMotor.getVelocity().getValueAsDouble()), getAngle());
  }

  public double getVoltage() {
    return mDriveMotor.getMotorVoltage().getValueAsDouble();
  }

  public SwerveModulePosition getPosition() {
    var driveRotations = mDriveMotor.getPosition().getValueAsDouble();
    // Calculate how many drive rotations were caused by azimuth coupling
    var azimuthCompensationDistance =
        getAngle().getRotations() * SwerveConstants.azimuthCouplingRatio;
    // Subtract the "false" rotations from the recorded rotations to get the rotations that caused
    // wheel motion
    var trueDriveRotations = driveRotations - azimuthCompensationDistance;

    return new SwerveModulePosition(motorRotToWheelMeter(trueDriveRotations), getAngle());
  }

  public static double wheelMeterToMotorRot(double wheelMeters) {
    return Conversions.metersToRotations(wheelMeters, SwerveConstants.wheelCircumference)
        * SwerveConstants.driveGearRatio;
  }

  public static double motorRotToWheelMeter(double motorRot) {
    return Conversions.rotationsToMeters(
        motorRot / SwerveConstants.driveGearRatio, SwerveConstants.wheelCircumference);
  }
}
