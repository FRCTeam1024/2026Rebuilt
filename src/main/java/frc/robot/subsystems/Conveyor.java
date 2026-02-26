package frc.robot.subsystems;

import static frc.robot.Constants.ConveyorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Conveyor extends SubsystemBase implements Logged {

  private final TalonFX motor = new TalonFX(motorID);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Conveyor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor.getMotorVoltage(),
        motor.getStatorCurrent(),
        motor.getSupplyCurrent(),
        motor.getSupplyVoltage(),
        motor.getVelocity());
    BaseStatusSignal.setUpdateFrequencyForAll(4, motor.getDeviceTemp());
    motor.optimizeBusUtilization();
    setOutput(0);
  }

  /**
   * Sets the output voltage of the climber as a proportion of the max output.
   *
   * @param output the proportion of max output [-1, 1]
   */
  public void setOutput(double output) {
    voltageRequest.Output = maxOutputVoltage * output;
    motor.setControl(voltageRequest);
  }

  public void stop() {
    setOutput(0);
  }

  public Command feedCommand() {
    return runEnd(
        () -> {
          setOutput(0.25);
        },
        () -> {
          stop();
        });
  }

  public Command ejectCommand() {
    return runEnd(
        () -> {
          setOutput(-0.3);
        },
        () -> {
          stop();
        });
  }

  /**
   * Creates a command that oscillates the conveyor, repeatedly going forward, off, reverse, then
   * off with predefined timing intervals.
   *
   * @return a command that repeatedly oscillates the conveyor
   */
  public Command oscillateCommand() {
    return run(() -> setOutput(oscillateForwardSpeed))
        .withTimeout(oscillateForwardTime)
        .andThen(run(() -> stop()).withTimeout(oscillateOffTime1))
        .andThen(run(() -> setOutput(oscillateReverseSpeed)).withTimeout(oscillateReverseTime))
        .andThen(run(() -> stop()).withTimeout(oscillateOffTime2))
        .repeatedly()
        .finallyDo(() -> stop());
  }

  @Override
  public void periodic() {
    log("Output Voltage", motor.getMotorVoltage().getValueAsDouble());
    log("Stator Current", motor.getStatorCurrent().getValueAsDouble());
    log("Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    log("Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
    log("Velocity", motor.getVelocity().getValueAsDouble());
    log("Requested Voltage", voltageRequest.Output);
    log("Temperature", motor.getDeviceTemp().getValueAsDouble());
  }
}
