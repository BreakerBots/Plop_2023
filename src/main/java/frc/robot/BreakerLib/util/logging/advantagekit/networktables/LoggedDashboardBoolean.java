package frc.robot.BreakerLib.util.logging.advantagekit.networktables;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLoggable;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;

public class LoggedDashboardBoolean implements LoggedDashboardInput {
  private final String key;
  private boolean defaultValue;
  private boolean value;

  private final BreakerLoggable inputs = new BreakerLoggable() {
    public void toLog(LogTable table) {
      table.put(key, value);
    }
  };

  /**
   * Creates a new LoggedDashboardBoolean, for handling a string input sent via
   * NetworkTables.
   * 
   * @param key The key for the boolean, published to
   *            "/SmartDashboard/{key}" for NT or
   *            "/DashboardInputs/{key}" when logged.
   */
  public LoggedDashboardBoolean(String key) {
    this(key, false);
  }

  /**
   * Creates a new LoggedDashboardBoolean, for handling a string input sent via
   * NetworkTables.
   * 
   * @param key          The key for the boolean, published to
   *                     "/SmartDashboard/{key}" for NT or
   *                     "/DashboardInputs/{key}" when logged.
   * @param defaultValue The default value if no value in NT is found.
   */
  public LoggedDashboardBoolean(String key, boolean defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.value = defaultValue;
    SmartDashboard.putBoolean(key, SmartDashboard.getBoolean(key, defaultValue));
    periodic();
    BreakerLog.getInstance().registerDashboardInput(this);
  }

  /** Updates the default value, which is used if no value in NT is found. */
  public void setDefault(boolean defaultValue) {
    this.defaultValue = defaultValue;
  }

  /**
   * Publishes a new value. Note that the value will not be returned by
   * {@link #get()} until the next cycle.
   */
  public void set(boolean value) {
    SmartDashboard.putBoolean(key, value);
  }

  /** Returns the current value. */
  public boolean get() {
    return value;
  }

  public void periodic() {
    value = SmartDashboard.getBoolean(key, defaultValue);
    BreakerLog.getInstance().processInputs(prefix, inputs);
  }
}