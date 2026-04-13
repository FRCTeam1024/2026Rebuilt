package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {

  private final TalonFX rightMotor = new TalonFX(rightMotorID);
  private final TalonFX leftMotor = new TalonFX(leftMotorID);

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private Debouncer jamDetectionFilter = new Debouncer(jamCurrentThresholdDurationSeconds);
  private boolean jamDetected;

  public Intake() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // may be worth opening the throttle on this, at the expense of more heating
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightMotor.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        rightMotor.getMotorVoltage(),
        rightMotor.getStatorCurrent(),
        rightMotor.getSupplyCurrent(),
        rightMotor.getSupplyVoltage(),
        rightMotor.getVelocity());
    BaseStatusSignal.setUpdateFrequencyForAll(4, rightMotor.getDeviceTemp());
    rightMotor.optimizeBusUtilization(0);

    leftMotor.optimizeBusUtilization(0);

    voltageRequest.UpdateFreqHz = 50;
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    rightMotor.setControl(voltageRequest);
  }

  public void setOutput(double output) {
    voltageRequest.Output = maxOutputVoltage * output;
    rightMotor.setControl(voltageRequest);
  }

  public void stop() {
    setOutput(0);
  }

  public Command intakeCommand() {
    return runEnd(
        () -> {
          setOutput(1.0);
        },
        () -> {
          stop();
        });
  }

  public Command ejectCommand() {
    return runEnd(
        () -> {
          setOutput(-1.0);
        },
        () -> {
          stop();
        });
  }

  public boolean jamDetected() {
    return jamDetected;
  }

  @Override
  public void periodic() {

    jamDetected =
        voltageRequest.Output != 0
            && jamDetectionFilter.calculate(
                rightMotor.getStatorCurrent().getValueAsDouble() >= jamCurrentThresholdAmps)
            && Math.abs(rightMotor.getVelocity().getValueAsDouble()) < jamVelocityThresholdRPS;

    log("Jam Detected", jamDetected);
    log("Output Voltage", rightMotor.getMotorVoltage().getValueAsDouble());
    log("Stator Current", rightMotor.getStatorCurrent().getValueAsDouble());
    log("Supply Current", rightMotor.getSupplyCurrent().getValueAsDouble());
    log("Supply Voltage", rightMotor.getSupplyVoltage().getValueAsDouble());
    log("Velocity", rightMotor.getVelocity().getValueAsDouble());
    log("Requested Voltage", voltageRequest.Output);
    log("Temperature", rightMotor.getDeviceTemp().getValueAsDouble());
  }
}
