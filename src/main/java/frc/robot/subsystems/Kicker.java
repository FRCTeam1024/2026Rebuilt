package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.KickerConstants.*;

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
import monologue.Logged;

public class Kicker extends SubsystemBase implements Logged {

  private final TalonFX upper = new TalonFX(upperShaftMotorID);
  private final TalonFX lower = new TalonFX(lowerShaftMotorID);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final CoastOut coastRequest = new CoastOut();
  private final StaticBrake estopRequest = new StaticBrake();

  public Kicker() {
    var config = new TalonFXConfiguration();
    // Universal configs
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Upper configs
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0.kS = 0.16265;
    config.Slot0.kV = 0.11711;
    config.Slot0.kA = 0.0035646;
    config.Slot0.kP = 0.164;

    upper.getConfigurator().apply(config);

    // Lower configs
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kS = 0.086639;
    config.Slot0.kV = 0.11848;
    config.Slot0.kA = 0.0029633;
    config.Slot0.kP = 0.1561;
    lower.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        upper.getSupplyCurrent(),
        lower.getSupplyCurrent(),
        upper.getStatorCurrent(),
        lower.getStatorCurrent(),
        upper.getVelocity(),
        lower.getVelocity(),
        upper.getMotorVoltage(),
        lower.getMotorVoltage(),
        upper.getSupplyVoltage(),
        lower.getSupplyVoltage());
    BaseStatusSignal.setUpdateFrequencyForAll(4, upper.getDeviceTemp(), lower.getDeviceTemp());

    // For sysID
    // BaseStatusSignal.setUpdateFrequencyForAll(
    //     100,
    //     upper.getVelocity(),
    //     upper.getMotorVoltage(),
    //     upper.getPosition(),
    //     lower.getVelocity(),
    //     lower.getMotorVoltage(),
    //     lower.getPosition());

    upper.optimizeBusUtilization(0);
    lower.optimizeBusUtilization(0);
    stop();
    voltageRequest.UpdateFreqHz = 50;
    velocityRequest.UpdateFreqHz = 50;
  }

  public void setControl(ControlRequest req) {
    upper.setControl(req);
    lower.setControl(req);
  }

  public void setVoltage(double output) {
    voltageRequest.Output = output;
    setControl(voltageRequest);
  }

  public void setVelocity(double velocity) {
    velocityRequest.Velocity = velocity;
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

  public boolean stopped() {
    return Math.abs(upper.getVelocity().getValueAsDouble()) < 0.01
        && Math.abs(lower.getVelocity().getValueAsDouble()) < 0.01;
  }

  public Command feedCommand() {
    return velocityCommand(() -> 41);
  }

  public Command retractCommand() {
    return velocityCommand(() -> -24);
  }

  public Command velocityCommand(DoubleSupplier velocity) {
    return runEnd(() -> setVelocity(velocity.getAsDouble()), this::stop);
  }

  private Command fastStopCommand() {
    return runOnce(this::emergencyStop).andThen(idle()).until(this::stopped);
  }

  public Command sysIdRoutine() {
    var routine = makeSysIdRoutine();
    return Commands.sequence(
        fastStopCommand(),
        routine.quasistatic(Direction.kForward).withTimeout(12),
        fastStopCommand(),
        routine.dynamic(Direction.kForward).withTimeout(5),
        fastStopCommand(),
        routine.quasistatic(Direction.kReverse).withTimeout(12),
        fastStopCommand(),
        routine.dynamic(Direction.kReverse).withTimeout(5),
        fastStopCommand());
  }

  private SysIdRoutine makeSysIdRoutine() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            Seconds.of(12),
            state -> SignalLogger.writeString("KickerSysIDState", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> {
              setVoltage(volts.in(Volts));
            },
            log -> {},
            this));
  }

  @Override
  public void periodic() {
    log("Requested Velocity", velocityRequest.Velocity);
    log("Requested Voltage", voltageRequest.Output);
    log("Upper Supply Current", upper.getSupplyCurrent().getValueAsDouble());
    log("Upper Stator Current", upper.getStatorCurrent().getValueAsDouble());
    log("Upper Velocity", upper.getVelocity().getValueAsDouble());
    log("Upper Applied Voltage", upper.getMotorVoltage().getValueAsDouble());
    log("Upper Supply Voltage", upper.getSupplyVoltage().getValueAsDouble());
    log("Upper Temperature", upper.getDeviceTemp().getValueAsDouble());
    log("Lower Supply Current", lower.getSupplyCurrent().getValueAsDouble());
    log("Lower Stator Current", lower.getStatorCurrent().getValueAsDouble());
    log("Lower Velocity", lower.getVelocity().getValueAsDouble());
    log("Lower Applied Voltage", lower.getMotorVoltage().getValueAsDouble());
    log("Lower Supply Voltage", lower.getSupplyVoltage().getValueAsDouble());
    log("Lower Temperature", lower.getDeviceTemp().getValueAsDouble());
  }
}
