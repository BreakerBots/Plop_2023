// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.BreakerGenericSwerveModuleMotor;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestableBase;

/** Add your docs here. */
public abstract class BreakerGenericSwerveModuleAngleMotor extends BreakerGenericSwerveModuleMotor {
    public abstract void setTargetAngle(Rotation2d targetAngle);
    public abstract Rotation2d getAbsoluteAngle();
    public abstract Rotation2d getRelativeAngle(); 
    public abstract void setBrakeMode(boolean isEnabled);
    public abstract Rotation2d getTargetAngle();
    public abstract BreakerSwerveModuleAngleMotorConfig getConfig();
    @Override
    public void toLog(LogTable table) {
        super.toLog(table);
        table.put("AbsoluteAngleDeg", getAbsoluteAngle().getDegrees());
        table.put("AbsoluteTargetAngleDeg", getTargetAngle().getDegrees());
        table.put("RelativeAngleDeg", getRelativeAngle().getDegrees());
        
    }

    public static class BreakerSwerveModuleAngleMotorConfig {
        private double azimuthGearRatio;
        private double supplyCurrentLimit;
        private BreakerSwerveMotorPIDConfig pidConfig;
        public BreakerSwerveModuleAngleMotorConfig(double azimuthGearRatio, double supplyCurrentLimit, BreakerSwerveMotorPIDConfig pidConfig) {
            this.azimuthGearRatio = azimuthGearRatio;
            this.supplyCurrentLimit = supplyCurrentLimit;
            this.pidConfig = pidConfig;
        }

        public double getAzimuthGearRatio() {
            return azimuthGearRatio;
        }

        public double getSupplyCurrentLimit() {
            return supplyCurrentLimit;
        }

        public BreakerSwerveMotorPIDConfig getPIDConfig() {
            return pidConfig;
        }
    }

}
