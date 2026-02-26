package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Climber extends SubsystemBase implements Logged {
  private final TalonFX motor = new TalonFX(motorID);

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final DigitalInput limitSwitch = new DigitalInput(limitSwitchPin);

  public Climber() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = retractLimit;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = extendLimit;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor.getMotorVoltage(),
        motor.getStatorCurrent(),
        motor.getSupplyCurrent(),
        motor.getSupplyVoltage(),
        motor.getPosition(),
        motor.getVelocity());

    BaseStatusSignal.setUpdateFrequencyForAll(4, motor.getDeviceTemp());
    motor.optimizeBusUtilization();
    motor.setPosition(0);

    // voltageRequest.UpdateFreqHz = 50;
    setOutput(0);
  }

  @Log(key = "At Bottom")
  public boolean isAtBottom() {
    return !limitSwitch.get();
  }

  /**
   * Sets the output voltage of the climber as a proportion of the max output.
   *
   * @param output the proportion of max output [-1, 1]
   */
  public void setOutput(double output) {
    voltageRequest.Output = maxOutputVoltage * output;
    voltageRequest.LimitForwardMotion = isAtBottom();
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
    log("Position", motor.getPosition().getValueAsDouble());
    log("Output Voltage", motor.getMotorVoltage().getValueAsDouble());
    log("Stator Current", motor.getStatorCurrent().getValueAsDouble());
    log("Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    log("Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
    log("Velocity", motor.getVelocity().getValueAsDouble());
    log("Requested Voltage", voltageRequest.Output);
    log("Temperature", motor.getDeviceTemp().getValueAsDouble());
  }
}
