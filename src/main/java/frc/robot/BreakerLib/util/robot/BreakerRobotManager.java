// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoManager;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerFalconOrchestra;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.logging.advantagekit.networktables.NT4Publisher;
import frc.robot.BreakerLib.util.logging.advantagekit.wpilog.WPILOGWriter;
import frc.robot.BreakerLib.util.test.selftest.SelfTest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.brakemode.BreakerAutoBrakeManager;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.brakemode.BreakerAutoBrakeManagerConfig;

/**
 * Robot manager that configures SelfTest functionality, automatic brake mode,
 * and auto paths.
 */
public class BreakerRobotManager {
    private static SelfTest test;
    private static BreakerAutoManager autoManager;
    private static BreakerAutoBrakeManager brakeModeManager;
    private static BreakerGenericDrivetrain baseDrivetrain;

    private BreakerRobotManager() {
    }

    /** Setup for the BreakerRobotManager.
     * 
     * @param baseDrivetrain Base drivetrain.
     * @param robotConfig Robot configuration.
     */
    public static void setup(BreakerGenericDrivetrain baseDrivetrain, BreakerRobotConfig robotConfig) {
        BreakerLog logger = BreakerLog.getInstance();
    
        // Set up data receivers & replay source
        if (RobotBase.isReal()) {
            logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
            logger.addDataReceiver(new NT4Publisher());
        } else {
            logger.addDataReceiver(new WPILOGWriter(""));
            logger.addDataReceiver(new NT4Publisher());
        }
        logger.start();
         test = new SelfTest(robotConfig.getSecondsBetweenSelfChecks(),
                    robotConfig.getAutoRegisterDevices());
        BreakerRobotManager.baseDrivetrain = baseDrivetrain;
        BreakerRobotManager.autoManager = robotConfig.usesPaths() ? new BreakerAutoManager(robotConfig.getAutoPaths())
                : new BreakerAutoManager();
        BreakerRobotManager.brakeModeManager = new BreakerAutoBrakeManager(
                new BreakerAutoBrakeManagerConfig(baseDrivetrain));
        BreakerLog.getInstance().logRobotStarted(robotConfig.getStartConfig());
    }

    /** @return Brake mode manager object. */
    public static BreakerAutoBrakeManager getBrakeModeManager() {
        return brakeModeManager;
    }

    /** @return SelfTest object. */
    public static SelfTest getSelfTest() {
        return test;
    }

    /** @return Auto manager object. */
    public static BreakerAutoManager getAutoManager() {
        return autoManager;
    }

    /** @return Autopath selected through auto manager. */
    public static Command getSelectedAutoPath() {
        return autoManager.getSelectedAutoPath();
    }

    /** Enable or disable brake mode. */
    public static void setDrivetrainBrakeMode(boolean isEnabled) {
        baseDrivetrain.setDrivetrainBrakeMode(isEnabled);
    }
}
