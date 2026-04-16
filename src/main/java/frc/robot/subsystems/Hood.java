package frc.robot.subsystems;

import static frc.robot.Constants.HoodConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterParameterCalculator;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class Hood extends SubsystemBase implements Logged {

  private final TalonFX motor = new TalonFX(motorID);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

  @Log private HomeState homeState = HomeState.Incomplete;
  private Timer homeStateChangeTimer = new Timer();
  private final Debouncer currentDebounce = new Debouncer(0.125, DebounceType.kRising);
  private final Debouncer velocityDebounce = new Debouncer(0.125, DebounceType.kRising);
  private final VoltageOut homeOutput = new VoltageOut(0);

  public Hood() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kA = kA;
    config.Slot0.kV = kV;
    config.Slot0.kG = kG;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = acceleration;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(minPosition);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(maxPosition);

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
    positionRequest.UpdateFreqHz = 50;
    // TODO: current homing
    motor.setPosition(0);
  }

  /**
   * Get the current hood position in degrees.
   *
   * @return The current hood position in degrees.
   */
  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  /**
   * Set the goal hood position in degrees.
   *
   * @param position The desired hood position in degrees.
   */
  public void setGoalPosition(double position) {

    if (position > 100) {
      position = 100;
    }
    positionRequest.Position = position;
    if (homeState == HomeState.Complete || homeState == HomeState.TimedOut) {
      motor.setControl(positionRequest);
    }
  }

  public Command setPositionCommand(DoubleSupplier position) {
    return run(() -> setGoalPosition(position.getAsDouble()));
  }

  public Command distanceCommand(DoubleSupplier distance) {
    return run(
        () ->
            setGoalPosition(
                ShooterParameterCalculator.calculateHub(distance.getAsDouble()).hoodPosition()));
  }

  public Command currentHome() {
    VoltageOut homeOutputRequest = new VoltageOut(homeOutputVolts);
    Debouncer currentDebounce = new Debouncer(homeCurrentDebounceSeconds, DebounceType.kRising);
    Debouncer velocityDebounce = new Debouncer(homeVelocityDebounceSeconds, DebounceType.kRising);

    return runOnce(
            () -> {
              currentDebounce.calculate(false);
              velocityDebounce.calculate(false);
              motor.setControl(homeOutputRequest.withOutput(homeOutputVolts));
            })
        .andThen(Commands.idle())
        .until(
            () ->
                currentDebounce.calculate(
                    motor.getStatorCurrent().getValueAsDouble() > homeCurrentThresholdAmps))
        .finallyDo(
            (interrupted) -> {
              motor.setControl(homeOutputRequest.withOutput(0));
              motor.setPosition(homePosition, 0);
              if (!interrupted) {
                motor.setControl(positionRequest.withPosition(stowPosition));
              }
            });
  }

  private boolean homeConditionMet() {
    var currentInRange =
        currentDebounce.calculate(
            motor.getStatorCurrent().getValueAsDouble() > homeCurrentThresholdAmps);
    return currentInRange;
  }

  private void setHomeState(HomeState state) {
    homeState = state;
    homeStateChangeTimer.restart();
  }

  private void runHomeStateMachine() {
    switch (homeState) {
      case Incomplete:
        if (DriverStation.isEnabled()) {
          motor.setControl(homeOutput.withOutput(homeOutputVolts));
          currentDebounce.calculate(false);
          velocityDebounce.calculate(false);
          setHomeState(HomeState.Running);
        }
        break;
      case Running:
        motor.setControl(homeOutput.withOutput(homeOutputVolts));
        if (!DriverStation.isEnabled()) {
          // We've disabled since starting. Go back to incomplete.
          motor.setControl(homeOutput.withOutput(0));
          setHomeState(HomeState.Incomplete);
        } else if (homeConditionMet()) {
          motor.setPosition(homePosition, 0);
          // go to last requested position
          motor.setControl(positionRequest);
          setHomeState((HomeState.Complete));
        } else if (homeStateChangeTimer.hasElapsed(3)) {
          motor.setPosition(0);
          motor.setControl(homeOutput.withOutput(0));
          DriverStation.reportError(
              "Initial hood homing timed out! Manual homing still available.", false);
          setHomeState(HomeState.TimedOut);
        }
        break;
      case Complete:
      case TimedOut:
        break;
    }
  }

  @Log(key = "At Goal")
  public boolean atGoal() {
    return MathUtil.isNear(positionRequest.Position, getPosition(), 0.5);
  }

  @Override
  public void periodic() {
    log("Setpoint", positionRequest.Position);
    log("Position", getPosition());
    log("Velocity", motor.getVelocity().getValueAsDouble());
    log("Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
    log("Stator Current", motor.getStatorCurrent().getValueAsDouble());
    log("Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    log("Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
    log("Temperature", motor.getDeviceTemp().getValueAsDouble());
    runHomeStateMachine();
  }

  public enum HomeState {
    Incomplete,
    Running,
    Complete,
    TimedOut
  }
}
