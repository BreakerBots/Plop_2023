// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.vision;

import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;

/** Add your docs here. */
public interface BreakerGenericVisionOdometer extends BreakerGenericOdometer {
    public double getDataTimestamp();
    public boolean isAnyTargetVisable();
}
