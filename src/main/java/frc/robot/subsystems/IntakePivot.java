package frc.robot.subsystems;

import static frc.robot.Constants.PivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class IntakePivot extends SubsystemBase implements Logged {
  private TalonFX motor = new TalonFX(motorID);
  private MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private VoltageOut homeRequest = new VoltageOut(0);

  public IntakePivot() {
    var config = new TalonFXConfiguration();
    // Intake mostly holds itself up in coast and brake is really annoying to move
    // manually
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = motorToPivotRatio;

    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = acceleration;

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
        motor.getSupplyVoltage());
    BaseStatusSignal.setUpdateFrequencyForAll(4, motor.getDeviceTemp());
    motor.optimizeBusUtilization();
    setZero();
    setGoal(0);
  }

  private void setHomingVoltage() {
    homeRequest.Output = homeOutputVolts;
    motor.setControl(homeRequest);
  }

  private void stopHoming() {
    homeRequest.Output = 0;
    motor.setControl(homeRequest);
    setZero();
    setGoal(0);
  }

  public void setGoal(double position) {
    positionRequest.Position = position;
    motor.setControl(positionRequest);
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  public Command setGoalCommand(double position) {
    return runOnce(() -> setGoal(position)).withName("Set Pivot Position");
  }

  public void setZero() {
    motor.setPosition(homePosition);
  }

  public Command currentHome() {
    VoltageOut homeOutputRequest = new VoltageOut(homeOutputVolts);
    StaticBrake brakeRequest = new StaticBrake();
    Debouncer currentDebounce = new Debouncer(homeCurrentDebounceSeconds, DebounceType.kRising);
    Debouncer velocityDebounce = new Debouncer(homeVelocityDebounceSeconds, DebounceType.kRising);

    return runOnce(
            () -> {
              currentDebounce.calculate(false);
              velocityDebounce.calculate(false);
              motor.setControl(homeOutputRequest);
            })
        .andThen(Commands.idle())
        .until(
            () ->
                currentDebounce.calculate(
                        motor.getStatorCurrent().getValueAsDouble() > homeCurrentThresholdAmps)
                    && velocityDebounce.calculate(
                        Math.abs(getVelocity()) < homeVelocityThresholdRPS))
        .finallyDo(
            (interrupted) -> {
              motor.setControl(brakeRequest);
              motor.setPosition(homePosition, 0);
              if (!interrupted) {
                motor.setControl(positionRequest.withPosition(homePosition));
              }
            });
  }

  @Override
  public void periodic() {
    log("Goal", positionRequest.Position);
    log("Position", motor.getPosition().getValueAsDouble());
    log("Velocity", motor.getVelocity().getValueAsDouble());
    log("Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
    log("Stator Current", motor.getStatorCurrent().getValueAsDouble());
    log("Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    log("Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
    log("Temperature", motor.getDeviceTemp().getValueAsDouble());
  }
}
