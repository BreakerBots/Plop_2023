// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors.smart;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.BreakerLib.devices.encoders.BreakerGenericEncoder;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLoggable;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestable;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Add your docs here. */
public interface BreakerGenericSmartMotorController extends BreakerSelfTestable, BreakerLoggable, MotorController, Sendable, AutoCloseable {

    public abstract double getDrawCurrent();
    public abstract void setBrakeMode();
    public default BreakerGenericEncoder getRotorEncoder() {
        return getEncoder(0);
    }
    public abstract BreakerGenericEncoder getSelectedEncoder();
    public abstract BreakerGenericEncoder getEncoder(int encoderNum);
    
    @Override
    default void toLog(LogTable table) {
        table.put("MotorOutput", get());
        table.put("DrawCurrentAmps", getDrawCurrent());
        table.put("DeviceHealth", getHealth().toString());
    }

    // public static enum SmartMotorControlMode {
    //     DUTY_CYCLE,
    //     VOLTAGE,
    //     POSITION,
    //     VELOCITY,
    //     CURRENT,
    //     SMART_MOTION
    // }

    // public static enum SmartMotorFeedforward {
    //     DUTY_CYCLE,
    //     VOLTAGE,
    //     POSITION,
    //     VELOCITY,
    //     CURRENT,
    //     SMART_MOTION
    // }
    
}
