package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {

  private final TalonFX left = new TalonFX(leftID);
  private final TalonFX right = new TalonFX(rightID);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public Shooter() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kP = 0.15597;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kS = 0.2; // tuned manually
    config.Slot0.kA = 0.035746;
    config.Slot0.kV = 0.1176470588; // tuned manually
    config.Slot0.kG = 0;

    left.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    right.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        left.getStatorCurrent(),
        right.getStatorCurrent(),
        left.getBridgeOutput(),
        right.getBridgeOutput(),
        left.getMotorOutputStatus(),
        right.getMotorOutputStatus(),
        left.getFaultField(),
        right.getFaultField());
    SmartDashboard.putNumber("ShooterVoltage", 4);
  }

  public void setVoltage(double output) {
    voltageRequest.Output = output;
    left.setControl(voltageRequest);
    right.setControl(voltageRequest);
  }

  public void setVelocity(double velocityRPS) {
    velocityRequest.Velocity = velocityRPS;
    left.setControl(velocityRequest);
    right.setControl(velocityRequest);
  }

  public void stop() {
    setVoltage(0);
  }

  public Command spinUpCommand(DoubleSupplier voltage) {
    return runEnd(
        () -> {
          setVelocity(voltage.getAsDouble());
        },
        () -> {
          stop();
        });
  }

  public Command spinUpCommand() {
    return spinUpCommand(() -> SmartDashboard.getNumber("ShooterVoltage", 0));
  }

  public Command sysIdRoutine() {
    var routine = makeRoutine();
    return Commands.sequence(
        routine.quasistatic(Direction.kForward).withTimeout(12),
        Commands.waitSeconds(5),
        routine.dynamic(Direction.kForward).withTimeout(5),
        Commands.waitSeconds(5),
        routine.quasistatic(Direction.kReverse).withTimeout(12),
        Commands.waitSeconds(5),
        routine.dynamic(Direction.kReverse).withTimeout(5),
        Commands.waitSeconds(5));
  }

  private SysIdRoutine makeRoutine() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("ShooterSysIDState", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> {
              setVoltage(volts.in(Volts));
            },
            log -> {},
            this));
  }

  @Override
  public void periodic() {
    log("Requested Voltage", voltageRequest.Output);
    log("Left Supply Current", left.getSupplyCurrent().getValueAsDouble());
    log("Right Supply Current", right.getSupplyCurrent().getValueAsDouble());
    log("Left Stator Current", left.getStatorCurrent().getValueAsDouble());
    log("Right Stator Current", right.getStatorCurrent().getValueAsDouble());
    log("Left Velocity", left.getVelocity().getValueAsDouble());
    log("Right Velocity", right.getVelocity().getValueAsDouble());
    log("Left Applied Voltage", left.getMotorVoltage().getValueAsDouble());
    log("Right Applied Voltage", right.getMotorVoltage().getValueAsDouble());
    log("Left Supply Voltage", left.getSupplyVoltage().getValueAsDouble());
    log("Right Supply Voltage", right.getSupplyVoltage().getValueAsDouble());
    log("Left Temperature", left.getDeviceTemp().getValueAsDouble());
    log("Right Temperature", right.getDeviceTemp().getValueAsDouble());
    log("Left output status", left.getMotorOutputStatus().toString());
    log("Right output status", right.getMotorOutputStatus().toString());
    log("Left bridge status", left.getBridgeOutput().toString());
    log("Right bridge status", right.getBridgeOutput().toString());
    log("Left overvoltage", left.getFault_OverSupplyV().getValue());
    log("Right overvoltage", right.getFault_OverSupplyV().getValue());
    log("Right faults", right.getFaultField().getValue());
    log("Left faults", left.getFaultField().getValue());
  }
}
