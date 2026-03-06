package frc.lib.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.DoubleSupplier;

/**
 * A utility class for creating tunable numbers that can be adjusted via NetworkTables.
 *
 * <p>Example usage:
 *
 * <pre>
 * var speed = new TunableNumber("speed", 0);
 * double currentSpeed = speed.get();
 * </pre>
 */
public class TunableNumber implements DoubleSupplier {
  private final String key;
  private final NetworkTableEntry entry;
  private double lastValue;

  /**
   * Creates a new TunableNumber with the specified key and default value.
   *
   * @param key The key to use in NetworkTables
   * @param defaultValue The default value to use if none is set
   */
  public TunableNumber(String key, double defaultValue) {
    this.key = key;

    this.entry = NetworkTableInstance.getDefault().getTable("Tuning").getEntry(key);
    entry.setDefaultDouble(defaultValue);
    lastValue = entry.getDouble(defaultValue);
  }

  /**
   * Gets the current value from NetworkTables.
   *
   * @return The current value
   */
  public double get() {
    double value = entry.getDouble(lastValue);
    lastValue = value;
    return value;
  }

  @Override
  public double getAsDouble() {
    return get();
  }

  /**
   * Gets the key used in NetworkTables.
   *
   * @return The key
   */
  public String getKey() {
    return key;
  }

  /**
   * Checks if the value has changed since the last get() call.
   *
   * @return true if the value has changed
   */
  public boolean hasChanged() {
    double currentValue = entry.getDouble(lastValue);
    return currentValue != lastValue;
  }
}
