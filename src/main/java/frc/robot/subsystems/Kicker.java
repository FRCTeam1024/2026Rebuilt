package frc.robot.subsystems;

import static frc.robot.Constants.KickerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Kicker extends SubsystemBase implements Logged {

  private final TalonFX leader = new TalonFX(leaderID);
  private final TalonFX follower = new TalonFX(followerID);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final Follower followRequest =
      new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed);

  public Kicker() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    leader.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    follower.getConfigurator().apply(config);

    follower.setControl(followRequest);
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
          setOutput(0.3);
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
    log("Leader Stator Current", leader.getStatorCurrent().getValueAsDouble());
    log("Follower Stator Current", follower.getStatorCurrent().getValueAsDouble());
    log("Leader Velocity", leader.getVelocity().getValueAsDouble());
    log("Follower Velocity", follower.getVelocity().getValueAsDouble());
    log("Leader Applied Voltage", leader.getMotorVoltage().getValueAsDouble());
    log("Follower Applied Voltage", follower.getMotorVoltage().getValueAsDouble());
    log("Leader Supply Voltage", leader.getSupplyVoltage().getValueAsDouble());
    log("Follower Supply Voltage", follower.getSupplyVoltage().getValueAsDouble());
    log("Leader Temperature", leader.getDeviceTemp().getValueAsDouble());
    log("Follower Temperature", follower.getDeviceTemp().getValueAsDouble());
  }
}
