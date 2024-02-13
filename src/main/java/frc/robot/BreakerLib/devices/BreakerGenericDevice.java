// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices;

import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLoggable;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestableBase;
import frc.robot.BreakerLib.util.test.selftest.SelfTest;

public abstract class BreakerGenericDevice extends BreakerSelfTestableBase implements BreakerLoggable {
    public BreakerGenericDevice() {
        SelfTest.autoRegisterDevice(this);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("DeviceHealth", getHealth().toString());
    }

}
