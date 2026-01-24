package frc.robot.subsystems;

import static frc.robot.Constants.KickerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Hood extends SubsystemBase implements Logged {

  private final TalonFX leader = new TalonFX(leaderID);
  private final TalonFX follower = new TalonFX(followerID);
  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final Follower followRequest =
      new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed);

  public Hood() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kP = 0.001;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kA = 0;
    config.Slot0.kV = 0;
    config.Slot0.kG = 0;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    leader.getConfigurator().apply(config);
    // TODO: current homing
    leader.setPosition(0);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    follower.getConfigurator().apply(config);

    follower.setControl(followRequest);
  }

  public void setPosition(double position) {
    positionRequest.Position = position;
    leader.setControl(positionRequest);
  }

  @Override
  public void periodic() {}
}
