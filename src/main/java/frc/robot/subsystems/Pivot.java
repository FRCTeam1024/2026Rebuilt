package frc.robot.subsystems;

import static frc.robot.Constants.PivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Pivot extends SubsystemBase implements Logged {
  private TalonFX motor = new TalonFX(motorID);
  private PositionVoltage positionRequest = new PositionVoltage(0);

  public Pivot() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxPosition;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // enable when limits are tuned
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minPosition;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kG = kG;
    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor.getPosition(),
        motor.getVelocity(),
        motor.getMotorVoltage(),
        motor.getStatorCurrent(),
        motor.getSupplyCurrent(),
        motor.getSupplyVoltage(),
        motor.getDeviceTemp());
  }

  public void setPosition(double position) {
    positionRequest.Position = position;
    motor.setControl(positionRequest);
  }

  public Command setPositionCommand(double position) {
    return run(() -> setPosition(position)).withName("Set Pivot Position");
  }

  @Override
  public void periodic() {
    log("Setpoint", positionRequest.Position);
    log("Position", motor.getPosition().getValueAsDouble());
    log("Velocity", motor.getVelocity().getValueAsDouble());
    log("Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
    log("Stator Current", motor.getStatorCurrent().getValueAsDouble());
    log("Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    log("Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
    log("Temperature", motor.getDeviceTemp().getValueAsDouble());
  }
}
