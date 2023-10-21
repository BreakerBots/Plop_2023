// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.BreakerGenericSwerveModuleDriveMotor;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;

/** Add your docs here. */
@Deprecated
public class BreakerLegacyFalconSwerveModuleDriveMotor extends BreakerGenericSwerveModuleDriveMotor {
    private BreakerSwerveModuleDriveMotorConfig config;
    private WPI_TalonFX motor;
    private double driveGearRatio, wheelDiameter, targetVelocity, maxAttainableWheelSpeed;
    private BreakerArbitraryFeedforwardProvider arbFF;
    public BreakerLegacyFalconSwerveModuleDriveMotor(WPI_TalonFX motor, boolean isMotorInverted, BreakerSwerveModuleDriveMotorConfig config) {
        this.motor = motor;
        this.config = config;
        this.driveGearRatio = config.getDriveGearRatio();
        this.wheelDiameter = Units.metersToInches(wheelDiameter);
        this.arbFF = config.getArbFF();
        maxAttainableWheelSpeed = config.getMaxAttainableWheelSpeed();
        targetVelocity = 0.0;
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        driveConfig.slot1.kP = config.getPIDConfig().kP;
        driveConfig.slot1.kI = config.getPIDConfig().kI;
        driveConfig.slot1.kD = config.getPIDConfig().kD;
        driveConfig.slot1.kF =config.getPIDConfig().kF;
        driveConfig.slot1.closedLoopPeakOutput = 1.0;
        driveConfig.peakOutputForward = 1.0;
        driveConfig.peakOutputReverse = -1.0;
        driveConfig.voltageCompSaturation = 12.0;
        driveConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, config.getSupplyCurrentLimit(), config.getSupplyCurrentLimit(), 1.5);
        BreakerPhoenix5Util.checkError(motor.configAllSettings(driveConfig),
                " Failed to config swerve module drive motor ");
        motor.selectProfileSlot(1, 0);
        motor.setInverted(isMotorInverted);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.set(ControlMode.Velocity, 0.0);
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        Pair<DeviceHealth, String> pair = BreakerPhoenix5Util.checkMotorFaultsAndConnection(motor);
        health = pair.getFirst();
        if (health != DeviceHealth.NOMINAL) {
            faultStr = " DRIVE_MOTOR_FAULTS : " + pair.getSecond();
        }
    }

    @Override
    public void setTargetVelocity(double targetMetersPerSecond, boolean isOpenLoop) {
        targetVelocity = targetMetersPerSecond;
        if (isOpenLoop) {
            motor.set(TalonFXControlMode.PercentOutput, targetMetersPerSecond / maxAttainableWheelSpeed);
        } else {
            motor.set(TalonFXControlMode.Velocity, getMetersPerSecToNativeVelUnits(targetMetersPerSecond),
            DemandType.ArbitraryFeedForward, arbFF.getArbitraryFeedforwardValue(targetMetersPerSecond));
        }
    }

    @Override
    public double getVelocity() {
        return Units.inchesToMeters(BreakerMath.ticksToInches(motor.getSelectedSensorVelocity() * 10,
        BreakerMath.getTicksPerInch(2048, driveGearRatio, wheelDiameter)));
    }

    @Override
    public double getDistance() {
        return Units.inchesToMeters(BreakerMath.ticksToInches(motor.getSelectedSensorPosition(),
            BreakerMath.getTicksPerInch(2048, driveGearRatio, wheelDiameter)));
    }

    @Override
    public void resetDistance() {
        motor.setSelectedSensorPosition(0);
    }

    @Override
    public void setBrakeMode(boolean isEnabled) {
        motor.setNeutralMode(isEnabled ? NeutralMode.Brake : NeutralMode.Coast);
        
    }

    @Override
    public double getTargetVelocity() {
        return targetVelocity;
    }

    private double getMetersPerSecToNativeVelUnits(double speedMetersPerSec) {
        return (speedMetersPerSec / 10) * Units.inchesToMeters(
                BreakerMath.getTicksPerInch(2048, driveGearRatio, wheelDiameter));
    }

    @Override
    public double getSupplyCurrent() {
        return motor.getSupplyCurrent();
    }

    @Override
    public double getMotorOutput() {
        return motor.getMotorOutputPercent();
    }

    @Override
    public BreakerSwerveModuleDriveMotorConfig getConfig() {
        return config;
    }
}
