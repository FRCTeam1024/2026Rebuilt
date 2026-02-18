package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Climber extends SubsystemBase implements Logged {
  private final TalonFX motor = new TalonFX(motorID);

  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Climber() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = retractLimit;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = extendLimit;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor.getMotorVoltage(),
        motor.getStatorCurrent(),
        motor.getSupplyCurrent(),
        motor.getSupplyVoltage(),
        motor.getVelocity(),
        motor.getDeviceTemp());
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

  public Command retractCommand() {
    return runEnd(() -> setOutput(retractOutput), () -> stop());
  }

  public Command extendCommand() {
    return runEnd(() -> setOutput(extendOutput), () -> stop());
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
