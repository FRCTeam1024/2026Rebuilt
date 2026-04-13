package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.ShooterParameterCalculator;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {

  private final TalonFX left = new TalonFX(leftID);
  private final TalonFX right = new TalonFX(rightID);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final CoastOut coastRequest = new CoastOut();
  private final StaticBrake estopRequest = new StaticBrake();

  public Shooter() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kP = 0.15597;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kS = 0.2; // tuned manually
    config.Slot0.kA = 0.035746;
    config.Slot0.kV = 0.1176470588; // tuned manually

    left.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    right.getConfigurator().apply(config);

    resetStatusFrequencies();
    left.optimizeBusUtilization(0);
    right.optimizeBusUtilization(0);

    voltageRequest.UpdateFreqHz = 50;
    velocityRequest.UpdateFreqHz = 50;
    stop();
  }

  private void setControl(ControlRequest request) {
    left.setControl(request);
    right.setControl(request);
  }

  private double getAverageVelocity() {
    return (left.getVelocity().getValueAsDouble() + right.getVelocity().getValueAsDouble()) / 2.0;
  }

  public void setVoltage(double output) {
    voltageRequest.Output = output;
    setControl(voltageRequest);
  }

  public void setVelocity(double velocityRPS) {
    setVelocity(velocityRPS, 0);
  }

  public void setVelocity(double velocityRPS, double accelerationRPS2) {
    velocityRequest.Velocity = velocityRPS;
    velocityRequest.Acceleration = accelerationRPS2;
    setControl(velocityRequest);
  }

  public void stop() {
    velocityRequest.Velocity = 0;
    setControl(coastRequest);
  }

  public void emergencyStop() {
    velocityRequest.Velocity = 0;
    setControl(estopRequest);
  }

  @Log(key = "At Setpoint")
  public boolean atSetpoint() {
    return velocityRequest.Velocity != 0
        && Math.abs(left.getVelocity().getValueAsDouble() - velocityRequest.Velocity)
            < velocityToleranceRPS
        && Math.abs(right.getVelocity().getValueAsDouble() - velocityRequest.Velocity)
            < velocityToleranceRPS;
  }

  public boolean stopped() {
    return Math.abs(left.getVelocity().getValueAsDouble()) < 0.01
        && Math.abs(right.getVelocity().getValueAsDouble()) < 0.01;
  }

  /**
   * Spin the shooter up to the given velocity. Ends when the shooter reaches the target velocity.
   * The shooter will not be stopped when the command ends.
   *
   * @param velocityRPS The target velocity in rotations per second.
   * @return A command that spins up the shooter.
   */
  public Command spinUpCommand(DoubleSupplier velocityRPS) {
    return run(() -> {
          setVelocity(velocityRPS.getAsDouble());
        })
        .until(this::atSetpoint);
  }

  /**
   * Run the shooter at the given velocity while the command is active, and stop when it ends.
   *
   * @param velocityRPS The target velocity in rotations per second.
   * @return A command that runs the shooter at the given velocity.
   */
  public Command velocityCommand(DoubleSupplier velocityRPS) {
    return runEnd(
        () -> {
          setVelocity(velocityRPS.getAsDouble());
        },
        () -> {
          stop();
        });
  }

  public Command distanceCommand(DoubleSupplier distance) {
    return runEnd(
        () -> {
          setVelocity(
              ShooterParameterCalculator.calculate(distance.getAsDouble()).shooterVelocity());
        },
        () -> {
          stop();
        });
  }

  /**
   * Idle the shooter, allowing it to slew up to the target velocity at a limited acceleration, and
   * stop when the command ends.
   *
   * @param velocityRPS
   * @return
   */
  public Command runIdleCommand(DoubleSupplier velocityRPS) {
    SlewRateLimiter velocitySlewRateLimiter = new SlewRateLimiter(velocitySlewRateRPSPerSecond);
    return startRun(
            () -> {
              velocitySlewRateLimiter.reset(getAverageVelocity());
            },
            () -> {
              double targetVelocity = velocityRPS.getAsDouble();
              double currentVelocity = velocitySlewRateLimiter.lastValue();
              double slewedVelocity = velocitySlewRateLimiter.calculate(targetVelocity);

              double acceleration = 0;
              if (slewedVelocity != targetVelocity) {
                acceleration =
                    slewedVelocity > currentVelocity
                        ? velocitySlewRateRPSPerSecond
                        : -velocitySlewRateRPSPerSecond;
              }

              setVelocity(slewedVelocity, acceleration);
            })
        .finallyDo(this::stop);
  }

  private Command fastStopCommand() {
    return runOnce(this::emergencyStop).andThen(idle()).until(this::stopped);
  }

  public Command sysIdRoutine() {
    var routine = makeSysIdRoutine();
    return Commands.sequence(
            runOnce(
                () -> {
                  // setStatusFrequenciesForSysId();
                }),
            fastStopCommand(),
            routine.quasistatic(Direction.kForward).withTimeout(12),
            fastStopCommand(),
            routine.dynamic(Direction.kForward).withTimeout(5),
            fastStopCommand(),
            routine.quasistatic(Direction.kReverse).withTimeout(12),
            fastStopCommand(),
            routine.dynamic(Direction.kReverse).withTimeout(5),
            fastStopCommand())
        .finallyDo(
            () -> {
              // resetStatusFrequencies();
            });
  }

  private SysIdRoutine makeSysIdRoutine() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            Seconds.of(12),
            state -> SignalLogger.writeString("ShooterSysIDState", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> {
              setVoltage(volts.in(Volts));
            },
            log -> {},
            this));
  }

  private void setStatusFrequenciesForSysId() {
    // TODO: test how long this takes
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        left.getMotorVoltage(),
        right.getMotorVoltage(),
        left.getVelocity(),
        right.getVelocity(),
        left.getPosition(),
        right.getPosition());
  }

  private void resetStatusFrequencies() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        left.getPosition(),
        right.getPosition(),
        left.getVelocity(),
        right.getVelocity(),
        left.getMotorVoltage(),
        right.getMotorVoltage());
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        right.getSupplyCurrent(),
        right.getSupplyCurrent(),
        left.getStatorCurrent(),
        right.getStatorCurrent(),
        left.getSupplyVoltage(),
        right.getSupplyVoltage());
    BaseStatusSignal.setUpdateFrequencyForAll(
        4,
        left.getDeviceTemp(),
        right.getDeviceTemp(),
        left.getFaultField(),
        right.getFaultField(),
        left.getBridgeOutput(),
        right.getBridgeOutput(),
        left.getMotorOutputStatus(),
        right.getMotorOutputStatus());
  }

  @Override
  public void periodic() {
    // log("Requested Voltage", voltageRequest.Output);
    log("Requested Velocity", velocityRequest.Velocity);
    log("Left Supply Current", left.getSupplyCurrent().getValueAsDouble());
    log("Right Supply Current", right.getSupplyCurrent().getValueAsDouble());
    log("Left Stator Current", left.getStatorCurrent().getValueAsDouble());
    log("Right Stator Current", right.getStatorCurrent().getValueAsDouble());
    log("Left Velocity", left.getVelocity().getValueAsDouble());
    log("Right Velocity", right.getVelocity().getValueAsDouble());
    log("Left Applied Voltage", left.getMotorVoltage().getValueAsDouble());
    log("Right Applied Voltage", right.getMotorVoltage().getValueAsDouble());
    log("Left Supply Voltage", left.getSupplyVoltage().getValueAsDouble());
    log("Right Supply Voltage", right.getSupplyVoltage().getValueAsDouble());
    log("Left Temperature", left.getDeviceTemp().getValueAsDouble());
    log("Right Temperature", right.getDeviceTemp().getValueAsDouble());
    log("Left output status", left.getMotorOutputStatus().toString());
    log("Right output status", right.getMotorOutputStatus().toString());
    log("Left bridge status", left.getBridgeOutput().toString());
    log("Right bridge status", right.getBridgeOutput().toString());
    log("Right faults", right.getFaultField().getValue());
    log("Left faults", left.getFaultField().getValue());
  }
}
