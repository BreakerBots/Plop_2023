package frc.robot.BreakerLib.util.logging.advantagekit.networktables;

import java.util.HashMap;
import java.util.Map;

import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.logging.advantagekit.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoggedDashboardChooser<V> implements LoggedDashboardInput {
  private final String key;
  private String selectedValue = null;
  private SendableChooser<String> sendableChooser = new SendableChooser<>();
  private Map<String, V> options = new HashMap<>();

  private final LoggableInputs inputs = new LoggableInputs() {
    public void toLog(LogTable table) {
      table.put(key, selectedValue);
    }
  };

  /**
   * Creates a new LoggedDashboardString, for handling a string input sent via
   * NetworkTables.
   * 
   * @param key The key for the chooser, published to
   *            "/SmartDashboard/{key}" for NT or
   *            "/DashboardInputs/{key}" when logged.
   */
  public LoggedDashboardChooser(String key) {
    this.key = key;
    SmartDashboard.putData(key, sendableChooser);
    periodic();
    BreakerLog.getInstance().registerDashboardInput(this);
  }

  /** Adds a new option to the chooser. */
  public void addOption(String key, V value) {
    sendableChooser.addOption(key, key);
    options.put(key, value);
  }

  /** Adds a new option to the chooser and sets it to the default. */
  public void addDefaultOption(String key, V value) {
    sendableChooser.setDefaultOption(key, key);
    options.put(key, value);
  }

  /**
   * Returns the selected option. If there is none selected, it will return the
   * default. If there is none selected and no default, then it will return
   * {@code null}.
   */
  public V get() {
    return options.get(selectedValue);
  }

  /**
   * Returns the internal sendable chooser object, for use when setting up
   * dashboard layouts. Do not read data from the sendable chooser directly.
   */
  public SendableChooser<String> getSendableChooser() {
    return sendableChooser;
  }

  public void periodic() {
    selectedValue = sendableChooser.getSelected();
    BreakerLog.getInstance().processInputs(prefix, inputs);
  }
}