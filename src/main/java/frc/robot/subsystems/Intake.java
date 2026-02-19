package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {

  private final TalonFX motor = new TalonFX(motorID);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Intake() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
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
          setOutput(0.7);
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

  @Override
  public void periodic() {
    // log("Output Voltage", motor.getMotorVoltage().getValueAsDouble());
    // log("Stator Current", motor.getStatorCurrent().getValueAsDouble());
    // log("Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    // log("Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
    // log("Velocity", motor.getVelocity().getValueAsDouble());
    log("Requested Voltage", voltageRequest.Output);
    // log("Temperature", motor.getDeviceTemp().getValueAsDouble());
  }
}
