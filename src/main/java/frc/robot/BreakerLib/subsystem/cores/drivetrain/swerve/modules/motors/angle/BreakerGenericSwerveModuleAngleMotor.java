// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.BreakerGenericSwerveModuleMotor;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestableBase;

/** Add your docs here. */
public abstract class BreakerGenericSwerveModuleAngleMotor extends BreakerGenericSwerveModuleMotor {
    public abstract void setTargetAngle(Rotation2d targetAngle);
    public abstract double getAbsoluteAngle();
    public abstract double getRelativeAngle(); 
    public abstract void setBrakeMode(boolean isEnabled);
    public abstract Rotation2d getTargetAngle();
    @Override
    public void toLog(LogTable table) {
        super.toLog(table);
        table.put("AbsoluteAngleDeg", getAbsoluteAngle());
        table.put("AbsoluteTargetAngleDeg", getTargetAngle().getDegrees());
        table.put("RelativeAngleDeg", getRelativeAngle());
        
    }

}
