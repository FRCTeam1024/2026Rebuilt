package frc.robot.subsystems;

import static frc.robot.Constants.KickerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Kicker extends SubsystemBase implements Logged {

  private final TalonFX upper = new TalonFX(upperShaftMotorID);
  private final TalonFX lower = new TalonFX(lowerShaftMotorID);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Kicker() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    upper.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
    upper.optimizeBusUtilization();
    lower.optimizeBusUtilization();
    setOutputVolts(0);
    voltageRequest.UpdateFreqHz = 50;
  }

  public void setControl(ControlRequest req) {
    upper.setControl(req);
    lower.setControl(req);
  }

  public void setOutputVolts(double output) {
    voltageRequest.Output = output;
    setControl(voltageRequest);
  }

  public void stop() {
    setOutputVolts(0);
  }

  public Command feedCommand() {
    return runEnd(
        () -> {
          setOutputVolts(5);
        },
        () -> {
          stop();
        });
  }

  public Command retractCommand() {
    return runEnd(
        () -> {
          setOutputVolts(-3);
        },
        () -> {
          stop();
        });
  }

  @Override
  public void periodic() {
    log("Requested Voltage", voltageRequest.Output);
    log("Left Supply Current", upper.getSupplyCurrent().getValueAsDouble());
    log("Left Stator Current", upper.getStatorCurrent().getValueAsDouble());
    log("Left Velocity", upper.getVelocity().getValueAsDouble());
    log("Left Applied Voltage", upper.getMotorVoltage().getValueAsDouble());
    log("Left Supply Voltage", upper.getSupplyVoltage().getValueAsDouble());
    log("Left Temperature", upper.getDeviceTemp().getValueAsDouble());
    log("Right Supply Current", lower.getSupplyCurrent().getValueAsDouble());
    log("Right Stator Current", lower.getStatorCurrent().getValueAsDouble());
    log("Right Velocity", lower.getVelocity().getValueAsDouble());
    log("Right Applied Voltage", lower.getMotorVoltage().getValueAsDouble());
    log("Right Supply Voltage", lower.getSupplyVoltage().getValueAsDouble());
    log("Right Temperature", lower.getDeviceTemp().getValueAsDouble());
  }
}
