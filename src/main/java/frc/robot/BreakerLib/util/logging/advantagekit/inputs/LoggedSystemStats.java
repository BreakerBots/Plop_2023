package frc.robot.BreakerLib.util.logging.advantagekit.inputs;
import frc.robot.BreakerLib.util.BreakerRoboRIO;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.logging.advantagekit.Logger;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Manages logging general system data.
 */
public class LoggedSystemStats {

  private static LoggedSystemStats instance;
  private static final Logger logger = Logger.getInstance();

  private final SystemStatsInputs sysInputs = new SystemStatsInputs();

  private LoggedSystemStats() {
  }

  public static LoggedSystemStats getInstance() {
    if (instance == null) {
      instance = new LoggedSystemStats();
    }
    return instance;
  }

  public static class SystemStatsInputs implements LoggableInputs {
    public double voltageVin;
    public double currentVin;
    public double userVoltage3v3;
    public double userCurrent3v3;
    public boolean userActive3v3;
    public int userCurrentFaults3v3;
    public double userVoltage5v;
    public double userCurrent5v;
    public boolean userActive5v;
    public int userCurrentFaults5v;
    public double userVoltage6v;
    public double userCurrent6v;
    public boolean userActive6v;
    public int userCurrentFaults6v;
    public boolean brownedOut;
    public boolean systemActive;
    public CANStatus canStatus = new CANStatus();
    public long epochTime;

    @Override
    public void toLog(LogTable table) {
      table.put("BatteryVoltage", voltageVin);
      table.put("BatteryCurrent", currentVin);

      table.put("3v3Rail/Voltage", userVoltage3v3);
      table.put("3v3Rail/Current", userCurrent3v3);
      table.put("3v3Rail/Active", userActive3v3);
      table.put("3v3Rail/CurrentFaults", userCurrentFaults3v3);

      table.put("5vRail/Voltage", userVoltage5v);
      table.put("5vRail/Current", userCurrent5v);
      table.put("5vRail/Active", userActive5v);
      table.put("5vRail/CurrentFaults", userCurrentFaults5v);

      table.put("6vRail/Voltage", userVoltage6v);
      table.put("6vRail/Current", userCurrent6v);
      table.put("6vRail/Active", userActive6v);
      table.put("6vRail/CurrentFaults", userCurrentFaults6v);

      table.put("BrownedOut", brownedOut);
      table.put("SystemActive", systemActive);

      table.put("CANBus/Utilization", canStatus.percentBusUtilization);
      table.put("CANBus/OffCount", canStatus.busOffCount);
      table.put("CANBus/TxFullCount", canStatus.txFullCount);
      table.put("CANBus/ReceiveErrorCount", canStatus.receiveErrorCount);
      table.put("CANBus/TransmitErrorCount", canStatus.transmitErrorCount);
      table.put("EpochTimeMicros", epochTime);
    }
  }

  public void periodic() {
    // Update inputs from conduit
      sysInputs.voltageVin = RoboRioDataJNI.getVInVoltage();
      sysInputs.currentVin = RoboRioDataJNI.getVInCurrent();

      sysInputs.userVoltage3v3 = RoboRioDataJNI.getUserVoltage3V3();
      sysInputs.userCurrent3v3 = RoboRioDataJNI.getUserCurrent3V3();
      sysInputs.userActive3v3 = RoboRioDataJNI.getUserActive3V3();
      sysInputs.userCurrentFaults3v3 = RoboRioDataJNI.getUserFaults3V3();

      sysInputs.userVoltage5v = RoboRioDataJNI.getUserVoltage5V();
      sysInputs.userCurrent5v = RoboRioDataJNI.getUserCurrent5V();
      sysInputs.userActive5v = RoboRioDataJNI.getUserActive5V();
      sysInputs.userCurrentFaults5v = RoboRioDataJNI.getUserFaults5V();

      sysInputs.userVoltage6v = RoboRioDataJNI.getUserVoltage6V();
      sysInputs.userCurrent6v = RoboRioDataJNI.getUserCurrent6V();
      sysInputs.userActive6v = RoboRioDataJNI.getUserActive6V();
      sysInputs.userCurrentFaults6v = RoboRioDataJNI.getUserFaults6V();

      sysInputs.brownedOut = RobotController.isBrownedOut();
      sysInputs.systemActive = RobotController.isSysActive();
      CANStatus canStatus = RobotController.getCANStatus();
      sysInputs.canStatus.setStatus(
          canStatus.percentBusUtilization,
          (int) canStatus.busOffCount,
          (int) canStatus.txFullCount,
          (int) canStatus.receiveErrorCount,
          (int) canStatus.transmitErrorCount);
      sysInputs.epochTime = RobotController.getFPGATime();

    logger.processInputs("SystemStats", sysInputs);
  }

  public SystemStatsInputs getInputs() {
    return sysInputs;
  }
}