// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive;

import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.BreakerGenericSwerveModuleMotor;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLoggable;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestableBase;

/** Add your docs here. */
public abstract class BreakerGenericSwerveModuleDriveMotor extends BreakerGenericSwerveModuleMotor {
    public abstract void setTargetVelocity(double targetMetersPerSeconde);
    public abstract double getVelocity();
    public abstract double getDistance();
    public abstract void resetDistance();
    public abstract void setBrakeMode(boolean isEnabled);
    public abstract double getTargetVelocity();
    @Override
    public void toLog(LogTable table) {
        super.toLog(table);
        table.put("VelocityMetersPerSec", getVelocity());
        table.put("TargetVelocityMetersPerSec", getTargetVelocity());
        table.put("WheelDistanceMeters", getDistance());
    }

}
