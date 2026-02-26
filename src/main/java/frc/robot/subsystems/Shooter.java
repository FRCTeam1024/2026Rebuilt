package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
    config.Slot0.kG = 0;

    left.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    right.getConfigurator().apply(config);

    resetStatusFrequencies();
    left.optimizeBusUtilization();
    right.optimizeBusUtilization();
    stop();
  }

  private void setControl(ControlRequest request) {
    left.setControl(request);
    right.setControl(request);
  }

  public void setVoltage(double output) {
    voltageRequest.Output = output;
    setControl(voltageRequest);
  }

  public void setVelocity(double velocityRPS) {
    velocityRequest.Velocity = velocityRPS;
    setControl(velocityRequest);
  }

  public void stop() {
    setControl(coastRequest);
  }

  public void emergencyStop() {
    setControl(estopRequest);
  }

  @Log(key = "At Setpoint")
  public boolean atSetpoint() {
    return Math.abs(left.getVelocity().getValueAsDouble() - velocityRequest.Velocity)
            < velocityToleranceRPS
        && Math.abs(right.getVelocity().getValueAsDouble() - velocityRequest.Velocity)
            < velocityToleranceRPS;
  }

  public Command spinUpCommand(DoubleSupplier velocityRPS) {
    return run(() -> {
          setVelocity(velocityRPS.getAsDouble());
        })
        .until(this::atSetpoint);
  }

  public Command velocityCommand(DoubleSupplier velocityRPS) {
    return runEnd(
        () -> {
          setVelocity(velocityRPS.getAsDouble());
        },
        () -> {
          stop();
        });
  }

  public Command sysIdRoutine() {
    var routine = makeSysIdRoutine();
    return Commands.sequence(
            runOnce(
                () -> {
                  setStatusFrequenciesForSysId();
                }),
            routine.quasistatic(Direction.kForward).withTimeout(12),
            waitSeconds(5),
            routine.dynamic(Direction.kForward).withTimeout(5),
            waitSeconds(5),
            routine.quasistatic(Direction.kReverse).withTimeout(12),
            waitSeconds(5),
            routine.dynamic(Direction.kReverse).withTimeout(5),
            waitSeconds(5))
        .finallyDo(
            () -> {
              resetStatusFrequencies();
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
        right.getSupplyCurrent(),
        right.getSupplyCurrent(),
        left.getStatorCurrent(),
        right.getStatorCurrent(),
        left.getVelocity(),
        right.getVelocity(),
        left.getMotorVoltage(),
        right.getMotorVoltage(),
        left.getSupplyVoltage(),
        right.getSupplyVoltage()
      );
    BaseStatusSignal.setUpdateFrequencyForAll(
        4,
        left.getDeviceTemp(),
        right.getDeviceTemp(),
        left.getBridgeOutput(),
        right.getBridgeOutput(),
        left.getMotorOutputStatus(),
        right.getMotorOutputStatus());
  }

  @Override
  public void periodic() {
    log("Requested Voltage", voltageRequest.Output);
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
