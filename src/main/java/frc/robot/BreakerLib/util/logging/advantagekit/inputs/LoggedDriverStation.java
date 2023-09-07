package frc.robot.BreakerLib.util.logging.advantagekit.inputs;

import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Manages logging and replaying data from the driver station (robot state,
 * joysticks, etc.)
 */
public class LoggedDriverStation {

  private static LoggedDriverStation instance;
  private static final BreakerLog logger = BreakerLog.getInstance();

  private final DriverStationInputs dsInputs = new DriverStationInputs();
  private final JoystickInputs[] joystickInputs = { new JoystickInputs(), new JoystickInputs(), new JoystickInputs(),
      new JoystickInputs(), new JoystickInputs(), new JoystickInputs() };
  private final MatchDataSender matchDataSender = new MatchDataSender();

  private LoggedDriverStation() {
  }

  public static LoggedDriverStation getInstance() {
    if (instance == null) {
      instance = new LoggedDriverStation();
    }
    return instance;
  }

  /**
   * General driver station data that needs to be updated throughout the match.
   */
  public static class DriverStationInputs implements LoggableInputs {
    public long allianceStation = 0;
    public String eventName = "";
    public String gameSpecificMessage = "";
    public long matchNumber = 0;
    public long replayNumber = 0;
    public long matchType = 0;
    public double matchTime = 0.00;

    public boolean enabled = false;
    public boolean autonomous = false;
    public boolean test = false;
    public boolean emergencyStop = false;
    public boolean fmsAttached = false;
    public boolean dsAttached = false;

    public void toLog(LogTable table) {
      table.put("AllianceStation", allianceStation);
      table.put("EventName", eventName);
      table.put("GameSpecificMessage", gameSpecificMessage);
      table.put("MatchNumber", matchNumber);
      table.put("ReplayNumber", replayNumber);
      table.put("MatchType", matchType);
      table.put("MatchTime", matchTime);

      table.put("Enabled", enabled);
      table.put("Autonomous", autonomous);
      table.put("Test", test);
      table.put("EmergencyStop", emergencyStop);
      table.put("FMSAttached", fmsAttached);
      table.put("DSAttached", dsAttached);
    }
  }

  /**
   * All of the required inputs for a single joystick.
   */
  public static class JoystickInputs implements LoggableInputs {
    public String name = "";
    public long type = 0;
    public boolean xbox = false;
    public long buttonCount = 0;
    public long buttonValues = 0;
    public long[] povs = {};
    public float[] axisValues = {};
    public long[] axisTypes = {};

    public void toLog(LogTable table) {
      table.put("Name", name);
      table.put("Type", type);
      table.put("Xbox", xbox);
      table.put("ButtonCount", buttonCount);
      table.put("ButtonValues", buttonValues);
      table.put("POVs", povs);
      table.put("AxisValues", axisValues);
      table.put("AxisTypes", axisTypes);
    }
  }

  /**
   * Records inputs from the real driver station via conduit
   */
  public void periodic() {
    // Update inputs 
      dsInputs.allianceStation = DriverStation.getLocation();
      dsInputs.eventName = DriverStation.getEventName().trim();
      dsInputs.gameSpecificMessage = DriverStation.getGameSpecificMessage();
      dsInputs.matchNumber = DriverStation.getMatchNumber();
      dsInputs.replayNumber = DriverStation.getReplayNumber();
      dsInputs.matchType = DriverStation.getMatchType().ordinal();
      dsInputs.matchTime = DriverStation.getMatchTime();

      dsInputs.enabled = DriverStation.isEnabled();
      dsInputs.autonomous = DriverStation.isAutonomous();
      dsInputs.test = DriverStation.isTest();
      dsInputs.emergencyStop = DriverStation.isEStopped();
      dsInputs.fmsAttached = DriverStation.isFMSAttached();
      dsInputs.dsAttached = DriverStation.isDSAttached();

      for (int id = 0; id < joystickInputs.length; id++) {
        JoystickInputs joystick = joystickInputs[id];
        joystick.name = DriverStation.getJoystickName(id).trim();
        joystick.type = DriverStation.getJoystickType(id);
        joystick.xbox = DriverStation.getJoystickIsXbox(id);
        joystick.buttonCount = DriverStation.getStickButtonCount(id);
        joystick.buttonValues = DriverStation.getStickButtons(id);

        // POVs
        int povCount = DriverStation.getStickPOVCount(id);
        long[] povValues = new long[povCount];
        for (int i = 0; i < povCount; i++) {
          povValues[i] = DriverStation.getStickPOV(id, i);
        }
        joystick.povs = povValues;
        // Axes
        int axisCount = DriverStation.getStickAxisCount(id);
        joystick.axisValues = new float[axisCount];
        joystick.axisTypes = new long[axisCount];
        for (int i = 0; i < axisCount; i++) {
          joystick.axisValues[i] = (float) DriverStation.getStickAxis(id, i);
          joystick.axisTypes[i] = DriverStation.getJoystickAxisType(id, i);
        }
    }

    // Send/receive log data
    logger.processInputs("DriverStation", dsInputs);
    for (int id = 0; id < joystickInputs.length; id++) {
      logger.processInputs("DriverStation/Joystick" + Integer.toString(id), joystickInputs[id]);
    }

    // Update FMSInfo table
    matchDataSender.sendMatchData(dsInputs);
  }

  /**
   * Returns a reference to an object containing all driver station data other
   * than joysticks.
   */
  public DriverStationInputs getDSData() {
    return dsInputs;
  }

  /**
   * Returns a reference to an object containing data for a single joystick.
   * 
   * @param id ID of the joystick to read(0-6)
   */
  public JoystickInputs getJoystickData(int id) {
    return joystickInputs[id];
  }

  /**
   * Class for updating the "FMSInfo" table in NetworkTables, modified from the
   * original DriverStation.
   */
  private static class MatchDataSender {
    NetworkTable table;
    StringPublisher typeMetadata;
    StringPublisher gameSpecificMessage;
    StringPublisher eventName;
    IntegerPublisher matchNumber;
    IntegerPublisher replayNumber;
    IntegerPublisher matchType;
    BooleanPublisher alliance;
    IntegerPublisher station;
    IntegerPublisher controlWord;
    boolean oldIsRedAlliance = true;
    long oldStationNumber = 1;
    String oldEventName = "";
    String oldGameSpecificMessage = "";
    long oldMatchNumber;
    long oldReplayNumber;
    long oldMatchType;
    long oldControlWord;

    MatchDataSender() {
      table = NetworkTableInstance.getDefault().getTable("FMSInfo");
      typeMetadata = table.getStringTopic(".type").publish();
      typeMetadata.set("FMSInfo");
      gameSpecificMessage = table.getStringTopic("GameSpecificMessage").publish();
      gameSpecificMessage.set("");
      eventName = table.getStringTopic("EventName").publish();
      eventName.set("");
      matchNumber = table.getIntegerTopic("MatchNumber").publish();
      matchNumber.set(0);
      replayNumber = table.getIntegerTopic("ReplayNumber").publish();
      replayNumber.set(0);
      matchType = table.getIntegerTopic("MatchType").publish();
      matchType.set(0);
      alliance = table.getBooleanTopic("IsRedAlliance").publish();
      alliance.set(true);
      station = table.getIntegerTopic("StationNumber").publish();
      station.set(1);
      controlWord = table.getIntegerTopic("FMSControlData").publish();
      controlWord.set(0);
    }

    private void sendMatchData(DriverStationInputs dsInputs) {
      boolean isRedAlliance = false;
      int stationNumber = 1;
      switch ((int) dsInputs.allianceStation) {
        case 0:
          isRedAlliance = true;
          stationNumber = 1;
          break;
        case 1:
          isRedAlliance = true;
          stationNumber = 2;
          break;
        case 2:
          isRedAlliance = true;
          stationNumber = 3;
          break;
        case 3:
          isRedAlliance = false;
          stationNumber = 1;
          break;
        case 4:
          isRedAlliance = false;
          stationNumber = 2;
          break;
        case 5:
          isRedAlliance = false;
          stationNumber = 3;
          break;
      }

      String currentEventName = dsInputs.eventName;
      String currentGameSpecificMessage = dsInputs.gameSpecificMessage;
      long currentMatchNumber = dsInputs.matchNumber;
      long currentReplayNumber = dsInputs.replayNumber;
      long currentMatchType = dsInputs.matchType;
      long currentControlWord = 0;
      currentControlWord += dsInputs.enabled ? 1 : 0;
      currentControlWord += dsInputs.autonomous ? 2 : 0;
      currentControlWord += dsInputs.test ? 4 : 0;
      currentControlWord += dsInputs.emergencyStop ? 8 : 0;
      currentControlWord += dsInputs.fmsAttached ? 16 : 0;
      currentControlWord += dsInputs.dsAttached ? 32 : 0;

      if (oldIsRedAlliance != isRedAlliance) {
        alliance.set(isRedAlliance);
        oldIsRedAlliance = isRedAlliance;
      }
      if (oldStationNumber != stationNumber) {
        station.set(stationNumber);
        oldStationNumber = stationNumber;
      }
      if (!oldEventName.equals(currentEventName)) {
        eventName.set(currentEventName);
        oldEventName = currentEventName;
      }
      if (!oldGameSpecificMessage.equals(currentGameSpecificMessage)) {
        gameSpecificMessage.set(currentGameSpecificMessage);
        oldGameSpecificMessage = currentGameSpecificMessage;
      }
      if (currentMatchNumber != oldMatchNumber) {
        matchNumber.set(currentMatchNumber);
        oldMatchNumber = currentMatchNumber;
      }
      if (currentReplayNumber != oldReplayNumber) {
        replayNumber.set(currentReplayNumber);
        oldReplayNumber = currentReplayNumber;
      }
      if (currentMatchType != oldMatchType) {
        matchType.set(currentMatchType);
        oldMatchType = currentMatchType;
      }
      if (currentControlWord != oldControlWord) {
        controlWord.set(currentControlWord);
        oldControlWord = currentControlWord;
      }
    }
  }
}
