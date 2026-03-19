package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {

  private final TalonFX motor = new TalonFX(motorID);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private Debouncer jamDetectionFilter = new Debouncer(jamCurrentThresholdDurationSeconds);
  private boolean jamDetected;

  public Intake() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // may be worth opening the throttle on this, at the expense of more heating
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor.getMotorVoltage(),
        motor.getStatorCurrent(),
        motor.getSupplyCurrent(),
        motor.getSupplyVoltage(),
        motor.getVelocity());
    BaseStatusSignal.setUpdateFrequencyForAll(4, motor.getDeviceTemp());
    voltageRequest.UpdateFreqHz = 50;
    motor.optimizeBusUtilization();
  }

  public void setOutput(double output) {
    voltageRequest.Output = maxOutputVoltage * output;
    motor.setControl(voltageRequest);
  }

  public void stop() {
    setOutput(0);
  }

  public Command intakeCommand() {
    return runEnd(
        () -> {
          setOutput(1.0);
        },
        () -> {
          stop();
        });
  }

  public Command ejectCommand() {
    return runEnd(
        () -> {
          setOutput(-1.0);
        },
        () -> {
          stop();
        });
  }

  public boolean jamDetected() {
    return jamDetected;
  }

  @Override
  public void periodic() {

    jamDetected =
        voltageRequest.Output != 0
            && jamDetectionFilter.calculate(
                motor.getStatorCurrent().getValueAsDouble() >= jamCurrentThresholdAmps)
            && Math.abs(motor.getVelocity().getValueAsDouble()) < jamVelocityThresholdRPS;

    log("Jam Detected", jamDetected);
    log("Output Voltage", motor.getMotorVoltage().getValueAsDouble());
    log("Stator Current", motor.getStatorCurrent().getValueAsDouble());
    log("Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    log("Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
    log("Velocity", motor.getVelocity().getValueAsDouble());
    log("Requested Voltage", voltageRequest.Output);
    log("Temperature", motor.getDeviceTemp().getValueAsDouble());
  }
}
