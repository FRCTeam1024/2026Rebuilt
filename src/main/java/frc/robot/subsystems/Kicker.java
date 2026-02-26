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

  private final TalonFX left = new TalonFX(leaderID);
  private final TalonFX right = new TalonFX(48);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Kicker() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    left.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    right.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        left.getSupplyCurrent(),
        right.getSupplyCurrent(),
        left.getStatorCurrent(),
        right.getStatorCurrent(),
        left.getVelocity(),
        right.getVelocity(),
        left.getMotorVoltage(),
        right.getMotorVoltage(),
        left.getSupplyVoltage(),
        right.getSupplyVoltage());
    BaseStatusSignal.setUpdateFrequencyForAll(4, left.getDeviceTemp(), right.getDeviceTemp());
    left.optimizeBusUtilization();
    right.optimizeBusUtilization();
    setOutput(0);
  }

  public void setControl(ControlRequest req) {
    left.setControl(req);
    right.setControl(req);
  }

  public void setOutput(double output) {
    voltageRequest.Output = maxOutputVoltage * output;
    setControl(voltageRequest);
  }

  public void stop() {
    setOutput(0);
  }

  public Command feedCommand() {
    return runEnd(
        () -> {
          setOutput(0.5);
        },
        () -> {
          stop();
        });
  }

  public Command retractCommand() {
    return runEnd(
        () -> {
          setOutput(-0.3);
        },
        () -> {
          stop();
        });
  }

  @Override
  public void periodic() {
    log("Requested Voltage", voltageRequest.Output);
    log("Left Supply Current", left.getSupplyCurrent().getValueAsDouble());
    log("Left Stator Current", left.getStatorCurrent().getValueAsDouble());
    log("Left Velocity", left.getVelocity().getValueAsDouble());
    log("Left Applied Voltage", left.getMotorVoltage().getValueAsDouble());
    log("Left Supply Voltage", left.getSupplyVoltage().getValueAsDouble());
    log("Left Temperature", left.getDeviceTemp().getValueAsDouble());
    log("Right Supply Current", right.getSupplyCurrent().getValueAsDouble());
    log("Right Stator Current", right.getStatorCurrent().getValueAsDouble());
    log("Right Velocity", right.getVelocity().getValueAsDouble());
    log("Right Applied Voltage", right.getMotorVoltage().getValueAsDouble());
    log("Right Supply Voltage", right.getSupplyVoltage().getValueAsDouble());
    log("Right Temperature", right.getDeviceTemp().getValueAsDouble());
  }
}
