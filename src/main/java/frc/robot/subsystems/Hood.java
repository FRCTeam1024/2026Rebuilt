package frc.robot.subsystems;

import static frc.robot.Constants.HoodConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Logged;

public class Hood extends SubsystemBase implements Logged {

  private final TalonFX motor = new TalonFX(motorID);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

  public Hood() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kA = kA;
    config.Slot0.kV = kV;
    config.Slot0.kG = kG;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = acceleration;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(minPosition);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(maxPosition);

    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor.getPosition(),
        motor.getVelocity(),
        motor.getMotorVoltage(),
        motor.getStatorCurrent(),
        motor.getSupplyCurrent(),
        motor.getSupplyVoltage());

    BaseStatusSignal.setUpdateFrequencyForAll(4, motor.getDeviceTemp());
    motor.optimizeBusUtilization();
    // TODO: current homing
    motor.setPosition(0);
  }

  /**
   * Get the current hood position in degrees.
   *
   * @return The current hood position in degrees.
   */
  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  /**
   * Set the goal hood position in degrees.
   *
   * @param position The desired hood position in degrees.
   */
  public void setGoalPosition(double position) {
    positionRequest.Position = position;
    motor.setControl(positionRequest);
  }

  public Command setPositionCommand(DoubleSupplier position) {
    return run(() -> setGoalPosition(position.getAsDouble()));
  }

  public Command currentHome() {
    VoltageOut homeOutputRequest = new VoltageOut(homeOutputVolts);
    StaticBrake brakeRequest = new StaticBrake();
    Debouncer currentDebounce = new Debouncer(homeCurrentDebounceSeconds, DebounceType.kRising);
    Debouncer velocityDebounce = new Debouncer(homeVelocityDebounceSeconds, DebounceType.kRising);

    return runOnce(
            () -> {
              currentDebounce.calculate(false);
              velocityDebounce.calculate(false);
              motor.setControl(homeOutputRequest);
            })
        .andThen(Commands.idle())
        .until(
            () ->
                currentDebounce.calculate(
                    motor.getStatorCurrent().getValueAsDouble() > homeCurrentThresholdAmps))
        .finallyDo(
            (interrupted) -> {
              motor.setControl(brakeRequest);
              motor.setPosition(homePosition, 0);
              if (!interrupted) {
                motor.setControl(positionRequest.withPosition(stowPosition));
              }
            });
  }

  @Override
  public void periodic() {
    log("Setpoint", positionRequest.Position);
    log("Position", getPosition());
    log("Velocity", motor.getVelocity().getValueAsDouble());
    log("Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
    log("Stator Current", motor.getStatorCurrent().getValueAsDouble());
    log("Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    log("Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
    log("Temperature", motor.getDeviceTemp().getValueAsDouble());
  }
}
