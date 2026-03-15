package frc.robot.subsystems;

import static frc.robot.Constants.ConveyorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import monologue.Logged;

public class Conveyor extends SubsystemBase implements Logged {

  private final TalonFX motor = new TalonFX(motorID);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final StatusSignal<Voltage> motorVoltage = motor.getMotorVoltage();
  private final StatusSignal<Voltage> supplyVoltage = motor.getSupplyVoltage();
  private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Temperature> deviceTemp = motor.getDeviceTemp();
  private final StatusSignalCollection signals =
      new StatusSignalCollection(
          motorVoltage, supplyVoltage, statorCurrent, supplyCurrent, velocity);

  public Conveyor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, motorVoltage, supplyVoltage, statorCurrent, supplyCurrent, velocity);
    deviceTemp.setUpdateFrequency(4);
    signals.addSignals(deviceTemp);
    motor.optimizeBusUtilization();

    voltageRequest.UpdateFreqHz = 50;
    setOutputVolts(0);
  }

  /**
   * Sets the output voltage of the conveyer.
   *
   * @param output output voltage
   */
  public void setOutputVolts(double output) {
    voltageRequest.Output = output;
    motor.setControl(voltageRequest);
  }

  public void stop() {
    setOutputVolts(0);
  }

  public Command feedCommand() {
    return runEnd(
        () -> {
          setOutputVolts(feedVolts);
        },
        () -> {
          stop();
        });
  }

  public Command ejectCommand() {
    return runEnd(
        () -> {
          setOutputVolts(ejectVolts);
        },
        () -> {
          stop();
        });
  }

  private Command clearJam() {
    return Commands.sequence(
        // Briefly hold
        idle().withTimeout(jamClearHoldTimeSeconds),
        // Clear the jam
        runEnd(() -> setOutputVolts(jamClearVolts), this::stop)
            .withTimeout(jamClearDurationSeconds));
  }

  /**
   * Continuously feeds the conveyor and automatically backdrives when a jam is detected via
   * sustained overcurrent.
   */
  public Command feedAutoJamClear() {
    // TODO: This may be better as a linear filter on the current itself depending on how noisy the
    // current is during a jam. It may also be worth making this a subsystem-wide value.
    Debouncer jamDetectionFilter = new Debouncer(jamThresholdDurationSeconds, DebounceType.kRising);
    BooleanSupplier jamDetected =
        () -> jamDetectionFilter.calculate(statorCurrent.getValueAsDouble() >= jamThresholdAmps);

    return Commands.sequence(
            // Continuously feed until a jam is detected.
            runEnd(() -> setOutputVolts(continuousFeedVolts), this::stop).until(jamDetected),
            clearJam())
        // rinse and repeat
        .repeatedly()
        .finallyDo(() -> stop());
  }

  /**
   * Creates a command that oscillates the conveyor, repeatedly going forward, off, reverse, then
   * off with predefined timing intervals.
   *
   * @return a command that repeatedly oscillates the conveyor
   */
  public Command oscillateCommand() {
    return run(() -> setOutputVolts(oscillateForwardVolts))
        .withTimeout(oscillateForwardTime)
        .andThen(run(() -> stop()).withTimeout(oscillateOffTime1))
        .andThen(run(() -> setOutputVolts(oscillateReverseVolts)).withTimeout(oscillateReverseTime))
        .andThen(run(() -> stop()).withTimeout(oscillateOffTime2))
        .repeatedly()
        .finallyDo(() -> stop());
  }

  @Override
  public void periodic() {
    signals.refreshAll();
    log("Output Voltage", motorVoltage.getValueAsDouble());
    log("Stator Current", statorCurrent.getValueAsDouble());
    log("Supply Current", supplyCurrent.getValueAsDouble());
    log("Supply Voltage", supplyVoltage.getValueAsDouble());
    log("Velocity", velocity.getValueAsDouble());
    log("Requested Voltage", voltageRequest.Output);
    log("Temperature", deviceTemp.getValueAsDouble());
  }
}
