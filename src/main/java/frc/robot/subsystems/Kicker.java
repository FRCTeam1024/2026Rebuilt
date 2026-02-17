package frc.robot.subsystems;

import static frc.robot.Constants.KickerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Kicker extends SubsystemBase implements Logged {

  private final TalonFX leader = new TalonFX(leaderID);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Kicker() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    leader.getConfigurator().apply(config);
  }

  public void setOutput(double output) {
    voltageRequest.Output = maxOutputVoltage * output;
    leader.setControl(voltageRequest);
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
    log("Leader Supply Current", leader.getSupplyCurrent().getValueAsDouble());
    log("Follower Supply Current", leader.getSupplyCurrent().getValueAsDouble());
    log("Leader Stator Current", leader.getStatorCurrent().getValueAsDouble());
    log("Leader Velocity", leader.getVelocity().getValueAsDouble());
    log("Leader Applied Voltage", leader.getMotorVoltage().getValueAsDouble());
    log("Leader Supply Voltage", leader.getSupplyVoltage().getValueAsDouble());
    log("Leader Temperature", leader.getDeviceTemp().getValueAsDouble());
  }
}
