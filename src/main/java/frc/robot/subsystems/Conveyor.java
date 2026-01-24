package frc.robot.subsystems;

import static frc.robot.Constants.ConveyorConstants.*;

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
  }

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
          setOutput(0.3);
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
}
