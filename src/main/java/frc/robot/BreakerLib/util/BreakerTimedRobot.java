package frc.robot.BreakerLib.util;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.NotifierJNI;

/**
 * BreakerTimedRobot implements the IterativeRobotBase robot program framework.
 *
 * <p>
 * The BreakerTimedRobot class is intended to be subclassed by a user creating a robot
 * program, and will call all required AdvantageKit periodic methods.
 *
 * <p>
 * periodic() functions from the base class are called on an interval by a
 * Notifier instance.
 */
public class BreakerTimedRobot extends IterativeRobotBase {

  public static final double defaultPeriodSecs = 0.02;
  private final int notifier = NotifierJNI.initializeNotifier();
  private final long periodUs;
  private long nextCycleUs = 0;

  private boolean useTiming = true;

  /** Constructor for LoggedRobot. */
  protected BreakerTimedRobot() {
    this(defaultPeriodSecs);
  }

  /**
   * Constructor for LoggedRobot.
   *
   * @param period Period in seconds.
   */
  protected BreakerTimedRobot(double period) {
    super(period);
    this.periodUs = (long) (period * 1000000);
    NotifierJNI.setNotifierName(notifier, "BreakerTimedRobot");

    HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Timed);
  }

  @Override
  @SuppressWarnings("NoFinalizer")
  protected void finalize() {
    NotifierJNI.stopNotifier(notifier);
    NotifierJNI.cleanNotifier(notifier);
  }

  /** Provide an alternate "main loop" via startCompetition(). */
  @Override
  @SuppressWarnings("UnsafeFinalization")
  public void startCompetition() {
    robotInit();

    if (isSimulation()) {
      simulationInit();
    }

    // Save data from init cycle
    BreakerLog.getInstance().periodicAfterUser();

    // Tell the DS that the robot is ready to be enabled
    System.out.println("********** Robot program startup complete **********");
    DriverStationJNI.observeUserProgramStarting();

    // Loop forever, calling the appropriate mode-dependent function
    while (true) {
      if (useTiming) {
        long currentTimeUs = BreakerLog.getInstance().getRealTimestamp();
        if (nextCycleUs < currentTimeUs) {
          // Loop overrun, start next cycle immediately
          nextCycleUs = currentTimeUs;
        } else {
          // Wait before next cycle
          NotifierJNI.updateNotifierAlarm(notifier, nextCycleUs);
          NotifierJNI.waitForNotifierAlarm(notifier);
        }
        nextCycleUs += periodUs;
      }

      long loopCycleStart = BreakerLog.getInstance().getRealTimestamp();
      BreakerLog.getInstance().periodicBeforeUser();
      long userCodeStart = BreakerLog.getInstance().getRealTimestamp();
      loopFunc();
      long loopCycleEnd = BreakerLog.getInstance().getRealTimestamp();
      BreakerLog.getInstance().recordOutput("BreakerTimedRobot/FullCycleMS", (loopCycleEnd - loopCycleStart) / 1000.0);
      BreakerLog.getInstance().recordOutput("BreakerTimedRobot/LogPeriodicMS", (userCodeStart - loopCycleStart) / 1000.0);
      BreakerLog.getInstance().recordOutput("BreakerTimedRobot/UserCodeMS", (loopCycleEnd - userCodeStart) / 1000.0);

      BreakerLog.getInstance().periodicAfterUser(); // Save data
    }
  }

  /** Ends the main loop in startCompetition(). */
  @Override
  public void endCompetition() {
    NotifierJNI.stopNotifier(notifier);
  }

  /** Sets whether to use standard timing or run as fast as possible. */
  public void setUseTiming(boolean useTiming) {
    this.useTiming = useTiming;
  }
}
