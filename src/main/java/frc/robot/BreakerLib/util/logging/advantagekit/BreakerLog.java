package frc.robot.BreakerLib.util.logging.advantagekit;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

import frc.robot.BreakerLib.util.logging.advantagekit.networktables.LoggedDashboardInput;
import frc.robot.BreakerLib.util.power.BreakerPowerDistribution;
import frc.robot.BreakerLib.util.robot.BreakerRobotStartConfig;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.BreakerLib.util.BreakerLibVersion;
import frc.robot.BreakerLib.util.BreakerRoboRIO.RobotOperatingMode;
import frc.robot.BreakerLib.util.logging.advantagekit.console.ConsoleSource;
import frc.robot.BreakerLib.util.logging.advantagekit.console.RIOConsoleSource;
import frc.robot.BreakerLib.util.logging.advantagekit.console.SimConsoleSource;
import frc.robot.BreakerLib.util.logging.advantagekit.inputs.LoggableInputs;
import frc.robot.BreakerLib.util.logging.advantagekit.inputs.LoggedDriverStation;
//import frc.robot.BreakerLib.util.logging.advantagekit.inputs.LoggedPowerDistribution;
import frc.robot.BreakerLib.util.logging.advantagekit.inputs.LoggedSystemStats;

/** Central class for recording and replaying log data. */
public class BreakerLog {
  private static final int receiverQueueCapcity = 500; // 10s at 50Hz

  private static BreakerLog instance;

  private boolean running = false;
  private LogTable entry = new LogTable(0);
  private LogTable outputTable;
  private LogTable messageTable;
  private Map<String, String> metadata = new HashMap<>();
  private ConsoleSource console;
  private List<LoggedDashboardInput> dashboardInputs = new ArrayList<>();
  private boolean deterministicTimestamps = true;
  private Map<String, BreakerLoggable> loggables = new HashMap<>();

  private final BlockingQueue<LogTable> receiverQueue = new ArrayBlockingQueue<LogTable>(receiverQueueCapcity);
  private final ReceiverThread receiverThread = new ReceiverThread(receiverQueue);
  private boolean receiverQueueFault = false;

  private BreakerLog() {
  }

  public static BreakerLog getInstance() {
    if (instance == null) {
      instance = new BreakerLog();
    }
    return instance;
  }

  /**
   * Adds a new data receiver to process real or replayed data. This method only
   * works during setup before starting to log.
   */
  public void addDataReceiver(LogDataReceiver dataReceiver) {
    if (!running) {
      receiverThread.addDataReceiver(dataReceiver);
    }
  }

  /**
   * Registers a new dashboard input to be included in the periodic loop. This
   * function should not be called by the user.
   */
  public void registerDashboardInput(LoggedDashboardInput dashboardInput) {
    dashboardInputs.add(dashboardInput);
  }

  public void registerLogable(String key, BreakerLoggable logable) {
    loggables.put(key, logable);
  }

  /**
   * Records a metadata value. This method only works during setup before starting
   * to log, then data will be recorded during the first cycle.
   * 
   * @param key   The name used to identify this metadata field.
   * @param value The value of the metadata field.
   */
  public void recordMetadata(String key, String value) {
    if (!running) {
      metadata.put(key, value);
    }
  }

  /**
   * Causes the timestamp returned by "Timer.getFPGATimestamp()" and similar to
   * match the "real" time as reported by the FPGA instead of the logged time from
   * AdvantageKit.
   * 
   * <p>
   * Not recommended for most users as the behavior of the replayed
   * code will NOT match the real robot. Only use this method if your control
   * logic requires precise timestamps WITHIN a single cycle and you have no way
   * to move timestamp-critical operations to an IO interface. Also consider using
   * "getRealTimestamp()" for logic that doesn't need to match the replayed
   * version (like for analyzing performance).
   */
  public void disableDeterministicTimestamps() {
    deterministicTimestamps = false;
  }

  /**
   * Starts running the logging system, including any data receivers or the replay
   * source.
   */
  public void start() {
    if (!running) {
      running = true;

      // Start console capture
      if (RobotBase.isReal()) {
        console = new RIOConsoleSource();
      } else {
        console = new SimConsoleSource();
      }


      // Create output table
      outputTable = entry.getSubtable("RealOutputs");
      messageTable = entry.getSubtable("Messages");

      // Record metadata
      LogTable metadataTable = entry.getSubtable("RealMetadata");
      for (Map.Entry<String, String> item : metadata.entrySet()) {
        metadataTable.put(item.getKey(), item.getValue());
      }

      // Start receiver thread
      receiverThread.start();

      // Start first periodic cycle
      periodicBeforeUser();
    }
  }

  /**
   * Ends the logging system, including any data receivers or the replay source.
   */
  public void end() {
    if (running) {
      running = false;
      try {
        console.close();
      } catch (Exception e) {
        DriverStation.reportError("Failed to stop console capture.", true);
      }
      receiverThread.interrupt();
    }
  }

  /**
   * Periodic method to be called before robotInit and each loop cycle. Updates
   * timestamp and globally logged data.
   */
  public void periodicBeforeUser() {
    if (running) {
      // Get next entry
      entry.setTimestamp(getRealTimestamp());

      // Update default inputs
      long saveDataStart = getRealTimestamp();
      LoggedDriverStation.getInstance().periodic();
      LoggedSystemStats.getInstance().periodic();
      BreakerPowerDistribution loggedPowerDistribution = BreakerPowerDistribution.getInstance();
      if (loggedPowerDistribution != null) {
        loggedPowerDistribution.periodic();
      }
      for (int i = 0; i < dashboardInputs.size(); i++) {
        dashboardInputs.get(i).periodic();
      }
      long saveDataEnd = getRealTimestamp();

      // Log output data
      recordOutput("Logger/SavePeriodicMS", (saveDataEnd - saveDataStart) / 1000.0);
      recordOutput("Logger/QueuedCycles", receiverQueue.size());
    } else {
      // Retrieve new data even if logger is disabled
      LoggedDriverStation.getInstance().periodic();
      //LoggedPowerDistribution loggedPowerDistribution = LoggedPowerDistribution.getInstance();
      // if (loggedPowerDistribution != null) {
      //   loggedPowerDistribution.periodic();
      // }
      LoggedSystemStats.getInstance().periodic();
    }
  }

  /**
   * Periodic method to be called after robotInit and each loop cycle. Sends data
   * to data receivers. Running this after user code allows IO operations to
   * occur between cycles rather than interferring with the main thread.
   */
  public void periodicAfterUser() {
    if (running) {
      try {
        for (Entry<String, BreakerLoggable> ent: loggables.entrySet()) {
          processInputs(ent.getKey(), ent.getValue());
        }
        // Update console output
        long consoleCaptureStart = getRealTimestamp();
        String consoleData = console.getNewData();
        if (!consoleData.isEmpty()) {
          messageTable.put("Console", consoleData.trim());
        }
        long consoleCaptureEnd = getRealTimestamp();
        recordOutput("Logger/ConsolePeriodicMS", (consoleCaptureEnd - consoleCaptureStart) / 1000.0);

        // Send a copy of the data to the receivers. The original object will be
        // kept and updated with the next timestamp (and new data if replaying).
        receiverQueue.add(LogTable.clone(entry));
        receiverQueueFault = false;
      } catch (IllegalStateException exception) {
        receiverQueueFault = true;
        DriverStation.reportError("Capacity of receiver queue exceeded, data will NOT be logged.", false);
      }
    }
  }

  /**
   * Returns the state of the receiver queue fault. This is tripped when the
   * receiver queue fills up, meaning that data is no longer being saved.
   */
  public boolean getReceiverQueueFault() {
    return receiverQueueFault;
  }

  /**
   * Returns the current FPGA timestamp or replayed time based on the current log
   * entry (microseconds).
   */
  public long getTimestamp() {
    if (!running || entry == null || !deterministicTimestamps) {
      return getRealTimestamp();
    } else {
      return entry.getTimestamp();
    }
  }

  /**
   * Returns the true FPGA timestamp in microseconds, regardless of the timestamp
   * used for logging. Useful for analyzing performance. DO NOT USE this method
   * for any logic which might need to be replayed.
   */
  public long getRealTimestamp() {
    return HALUtil.getFPGATime();
  }

  /**
   * Processes a set of inputs, logging them on the real robot or updating them in
   * the simulator. This should be called every loop cycle after updating the
   * inputs from the hardware (if applicable).
   * 
   * @param key    The name used to identify this set of inputs.
   * @param inputs The inputs to log or update.
   */
  public void processInputs(String key, LoggableInputs inputs) {
    if (running) {
      inputs.toLog(entry.getSubtable(key));
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, byte[] value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, boolean value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, long value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, float value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, double value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, String value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, boolean[] value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, long[] value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, float[] value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, double[] value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, String[] value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * The poses are logged as a double array (x_1, y_1, rot_1, ...)
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, Pose2d... value) {
    double[] data = new double[value.length * 3];
    for (int i = 0; i < value.length; i++) {
      data[i * 3] = value[i].getX();
      data[i * 3 + 1] = value[i].getY();
      data[i * 3 + 2] = value[i].getRotation().getRadians();
    }
    recordOutput(key, data);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * The poses are logged as a double array (x, y, z, w_rot, x_rot, y_rot,
   * z_rot, ...)
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, Pose3d... value) {
    double[] data = new double[value.length * 7];
    for (int i = 0; i < value.length; i++) {
      data[i * 7] = value[i].getX();
      data[i * 7 + 1] = value[i].getY();
      data[i * 7 + 2] = value[i].getZ();
      data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
      data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
      data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
      data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
    }
    recordOutput(key, data);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * The trajectory is logged as a series of poses.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, Trajectory value) {
    recordOutput(key, value.getStates().stream().map(state -> state.poseMeters).toArray(Pose2d[]::new));
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * The modules are logged as a double array (angle_1, speed_1, angle_2, speed_2,
   * ...)
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, SwerveModuleState... value) {
    double[] data = new double[value.length * 2];
    for (int i = 0; i < value.length; i++) {
      data[i * 2] = value[i].angle.getRadians();
      data[i * 2 + 1] = value[i].speedMetersPerSecond;
    }
    recordOutput(key, data);
  }

  // /**
  //  * Records a single output field for easy access when viewing the log. On the
  //  * simulator, use this method to record extra data based on the original inputs.
  //  * 
  //  * The current position of the Mechanism2d is logged once as a set of nested
  //  * fields. If the position is updated, this method must be called again.
  //  * 
  //  * @param key   The name of the field to record. It will be stored under
  //  *              "/RealOutputs" or "/ReplayOutputs"
  //  * @param value The value of the field.
  //  */
  // public void recordOutput(String key, Mechanism2d value) {
  //   if (running) {
  //     try {
  //       // Use reflection because we don't explicitly depend on the shimmed classes
  //       Mechanism2d.class.getMethod("akitLog", LogTable.class).invoke(value, outputTable.getSubtable(key));
  //     } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException | NoSuchMethodException
  //         | SecurityException e) {
  //       e.printStackTrace();
  //     }
  //   }
  // }

   /** Startup message for robot. */
   public void logRobotStarted(BreakerRobotStartConfig startConfig) {
    StringBuilder work = new StringBuilder(" | ---------------- ROBOT STARTED ---------------- |\n\n");
    work.append(" TEAM: " + startConfig.getTeamNum() + " - " + startConfig.getTeamName() + "\n");
    work.append(" ROBOT: " + startConfig.getRobotName() + " - " + startConfig.getRobotYear() + "\n");
    work.append(" BREAKERLIB: " + BreakerLibVersion.Version + " | " + "ROBOT SOFTWARE: "
        + startConfig.getRobotSoftwareVersion() + "\n");
    work.append(" AUTHORS: " + startConfig.getAuthorNames() + "\n\n");
    work.append(" | ---------------------------------------------- | \n\n\n");
    logMessage(work.toString());
    recordMetadata("Team", Integer.toString(startConfig.getTeamNum()));
    recordMetadata("Robot", startConfig.getRobotName());
    recordMetadata("BreakerLibVersion", BreakerLibVersion.Version);
    recordMetadata("RobotSoftware", startConfig.getRobotSoftwareVersion());
    recordMetadata("Authors", startConfig.getAuthorNames());
  }

  /**
   * Logs robot mode change and plays enable tune. (automatically called by
   * BreakerRoboRIO)
   */
  public void logRobotChangedMode(RobotOperatingMode newMode) {
    logMessage("| ---- ROBOT MODE CHANGED TO: " + newMode + " ---- |");
  }

  /** Logs given event. */
  public void logEvent(String event) {
    String message = " EVENT: " + event;
    if (running) {
      messageTable.put("Events", message);
    }
    System.out.println(message);
  }

  /**
   * Internal logging function for breakerlib classes to sepreate automated
   * breakerlib logging from user loging
   */
  public void logBreakerLibEvent(String event) {
    String message = " BREAKERLIB INTERNAL EVENT: " + event;
    if (running) {
      messageTable.put("BreakerLibInternalEvents", message);
    }
    System.out.println(message);
  }

  /**
   * Logs either exceptions thrown by code, or suer difigned errors either from
   * code or physical errors
   */
  public void logError(String error) {
    String message = " ERROR: " + error;
    if (running) {
      messageTable.put("Errors", message);
    }
    System.out.println(message);
  }

  public void logError(Exception e) {
    logError(" ERROR: " + e.toString() + " : " + e.getStackTrace().toString());
  }

  /**
   * Logs robot superstructure (physical) events (i.e. intake activated, shooter
   * enabled)
   */
  public void logSuperstructureEvent(String event) {
    String message = " ROBOT SUPERSTRUCTURE EVENT: " + event;
    if (running) {
      messageTable.put("SuperstructureEvents", message);
    }
    System.out.println(message);
  }

  /** Write custom message to log. */
  public void logMessage(String message) {
    if (running) {
      messageTable.put("Messages", message);
    }
    System.out.println(message);
  }

}
