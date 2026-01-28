package frc.robot.subsystems;

import static frc.robot.Constants.HoodConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Logged;

public class Hood extends SubsystemBase implements Logged {

  private final TalonFX leader = new TalonFX(leaderID);
  private final TalonFX follower = new TalonFX(followerID);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final Follower followRequest =
      new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed);

  public Hood() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kP = 10;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kA = 0;
    config.Slot0.kV = 0;
    config.Slot0.kG = 0;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.MotionMagic.MotionMagicCruiseVelocity = 10;
    config.MotionMagic.MotionMagicAcceleration = 100;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.7;

    leader.getConfigurator().apply(config);
    // TODO: current homing
    leader.setPosition(0);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    follower.getConfigurator().apply(config);
    // follower.setControl(followRequest);
  }

  public void setPosition(double position) {
    positionRequest.Position = MathUtil.clamp(position, minPosition, maxPosition);
    leader.setControl(positionRequest);
  }

  public Command setPositionCommand(DoubleSupplier position) {
    return run(() -> setPosition(position.getAsDouble()));
  }

  @Override
  public void periodic() {}
}
