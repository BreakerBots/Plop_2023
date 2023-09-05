package frc.robot.BreakerLib.util.logging.advantagekit.networktables;

import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.logging.advantagekit.Logger;
import frc.robot.BreakerLib.util.logging.advantagekit.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoggedDashboardString implements LoggedDashboardInput {
  private final String key;
  private String defaultValue;
  private String value;

  private final LoggableInputs inputs = new LoggableInputs() {
    public void toLog(LogTable table) {
      table.put(key, value);
    }
  };

  /**
   * Creates a new LoggedDashboardString, for handling a string input sent via
   * NetworkTables.
   * 
   * @param key The key for the string, published to
   *            "/SmartDashboard/{key}" for NT or
   *            "/DashboardInputs/{key}" when logged.
   */
  public LoggedDashboardString(String key) {
    this(key, "");
  }

  /**
   * Creates a new LoggedDashboardString, for handling a string input sent via
   * NetworkTables.
   * 
   * @param key          The key for the string, published to
   *                     "/SmartDashboard/{key}" for NT or
   *                     "/DashboardInputs/{key}" when logged.
   * @param defaultValue The default value if no value in NT is found.
   */
  public LoggedDashboardString(String key, String defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.value = defaultValue;
    SmartDashboard.putString(key, SmartDashboard.getString(key, defaultValue));
    periodic();
    Logger.getInstance().registerDashboardInput(this);
  }

  /** Updates the default value, which is used if no value in NT is found. */
  public void setDefault(String defaultValue) {
    this.defaultValue = defaultValue;
  }

  /**
   * Publishes a new value. Note that the value will not be returned by
   * {@link #get()} until the next cycle.
   */
  public void set(String value) {
    SmartDashboard.putString(key, value);
  }

  /** Returns the current value. */
  public String get() {
    return value;
  }

  public void periodic() {
    value = SmartDashboard.getString(key, defaultValue);
    Logger.getInstance().processInputs(prefix, inputs);
  }
}