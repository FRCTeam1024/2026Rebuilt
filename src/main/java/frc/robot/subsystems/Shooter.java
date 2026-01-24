package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {

  private final TalonFX leader = new TalonFX(leaderID);
  private final TalonFX follower = new TalonFX(followerID);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final Follower followRequest =
      new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed);

  public Shooter() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    leader.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    follower.getConfigurator().apply(config);

    follower.setControl(followRequest);
  }

  public void setVoltage(double output) {
    voltageRequest.Output = output;
    leader.setControl(voltageRequest);
  }

  public void stop() {
    setVoltage(0);
  }

  public Command spinUpCommand(DoubleSupplier voltage) {
    return runEnd(
        () -> {
          setVoltage(voltage.getAsDouble());
        },
        () -> {
          stop();
        });
  }
}
