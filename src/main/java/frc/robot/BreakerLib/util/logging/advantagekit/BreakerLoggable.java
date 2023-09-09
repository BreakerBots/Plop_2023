package frc.robot.BreakerLib.util.logging.advantagekit;

/**
 * A set of values which can be logged and replayed (for example, the hardware
 * inputs for a subsystem). Data is stored in LogTable objects.
 */
public interface BreakerLoggable {
  /**
   * Updates a LogTable with the data to log.
   */
  public void toLog(LogTable table);

}
