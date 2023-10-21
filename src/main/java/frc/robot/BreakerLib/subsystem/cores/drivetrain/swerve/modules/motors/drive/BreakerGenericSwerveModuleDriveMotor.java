// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive;

import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.BreakerGenericSwerveModuleMotor;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLoggable;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestableBase;

/** Add your docs here. */
public abstract class BreakerGenericSwerveModuleDriveMotor extends BreakerGenericSwerveModuleMotor {
    public abstract void setTargetVelocity(double targetMetersPerSeconde, boolean isOpenLoop);
    public abstract double getVelocity();
    public abstract double getDistance();
    public abstract void resetDistance();
    public abstract void setBrakeMode(boolean isEnabled);
    public abstract double getTargetVelocity();
    public abstract BreakerSwerveModuleDriveMotorConfig getConfig();
    @Override
    public void toLog(LogTable table) {
        super.toLog(table);
        table.put("VelocityMetersPerSec", getVelocity());
        table.put("TargetVelocityMetersPerSec", getTargetVelocity());
        table.put("WheelDistanceMeters", getDistance());
    }

    public static class BreakerSwerveModuleDriveMotorConfig {
        private double driveGearRatio;
        private double wheelDiameter;
        private double supplyCurrentLimit;
        private double maxAttainableWheelSpeed;
        private BreakerArbitraryFeedforwardProvider arbFF;
        private BreakerSwerveMotorPIDConfig pidConfig;
        public BreakerSwerveModuleDriveMotorConfig(double driveGearRatio, double wheelDiameter, double supplyCurrentLimit, double maxAttainableWheelSpeed, BreakerArbitraryFeedforwardProvider arbFF, BreakerSwerveMotorPIDConfig pidConfig) {
            this.arbFF = arbFF;
            this.driveGearRatio = driveGearRatio;
            this.pidConfig = pidConfig;
            this.supplyCurrentLimit = supplyCurrentLimit;
            this.wheelDiameter = wheelDiameter;
            this.maxAttainableWheelSpeed = maxAttainableWheelSpeed;
        }

        public BreakerArbitraryFeedforwardProvider getArbFF() {
            return arbFF;
        }

        public double getDriveGearRatio() {
            return driveGearRatio;
        }

        public BreakerSwerveMotorPIDConfig getPIDConfig() {
            return pidConfig;
        }

        public double getSupplyCurrentLimit() {
            return supplyCurrentLimit;
        }

        public double getWheelDiameter() {
            return wheelDiameter;
        }

        public double getMaxAttainableWheelSpeed() {
            return maxAttainableWheelSpeed;
        }
    }

}
