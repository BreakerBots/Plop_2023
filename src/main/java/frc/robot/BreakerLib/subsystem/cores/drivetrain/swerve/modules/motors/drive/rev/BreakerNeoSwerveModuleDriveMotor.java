// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.BreakerGenericSwerveModuleDriveMotor;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerREVUtil;

/** Add your docs here. */
public class BreakerNeoSwerveModuleDriveMotor extends BreakerGenericSwerveModuleDriveMotor {
    private CANSparkMax motor;
    private double driveGearRatio, wheelDiameter, targetVelocity;
    private BreakerArbitraryFeedforwardProvider arbFF;
    public BreakerNeoSwerveModuleDriveMotor(CANSparkMax motor, double driveGearRatio, double wheelDiameter, int supplyCurrentLimit, boolean isMotorInverted, BreakerArbitraryFeedforwardProvider arbFF, BreakerSwerveMotorPIDConfig pidConfig) {
        this.motor = motor;
        this.driveGearRatio = driveGearRatio;
        this.wheelDiameter = wheelDiameter;
        this.arbFF = arbFF;
        targetVelocity = 0.0;
        SparkMaxPIDController drivePID = motor.getPIDController();
        drivePID.setP(pidConfig.kP);
        drivePID.setI(pidConfig.kI);
        drivePID.setD(pidConfig.kD);
        drivePID.setFF(pidConfig.kF);

        BreakerREVUtil.checkError(motor.enableVoltageCompensation(12.0), "Failed to config " + deviceName + " voltage compensation");
        BreakerREVUtil.checkError(motor.setSmartCurrentLimit(supplyCurrentLimit),  "Failed to config " + deviceName + " smart current limit");
        motor.setInverted(isMotorInverted);
        motor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        Pair<DeviceHealth, String> pair = BreakerREVUtil.getSparkMaxHealthAndFaults(motor.getFaults());
        health = pair.getFirst();
        if (health != DeviceHealth.NOMINAL) {
            faultStr = " DRIVE_MOTOR_FAULTS : " + pair.getSecond();
        }
    }

    @Override
    public void setTargetVelocity(double targetMetersPerSecond) {
        targetVelocity = targetMetersPerSecond;
        motor.getPIDController().setReference(getMetersPerSecToNativeVelUnits(targetMetersPerSecond),
                CANSparkMax.ControlType.kVelocity, 0, arbFF.getArbitraryFeedforwardValue(targetMetersPerSecond),
                SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }

    @Override
    public double getVelocity() {
        return ((motor.getEncoder().getVelocity() / driveGearRatio) * (wheelDiameter * Math.PI)) / 60.0;
    }

    @Override
    public double getDistance() {
        return ((motor.getEncoder().getPosition() / driveGearRatio) * (wheelDiameter * Math.PI)) / 60.0;
    }

    @Override
    public void resetDistance() {
        motor.getEncoder().setPosition(0);
    }

    @Override
    public void setBrakeMode(boolean isEnabled) {
        motor.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public double getTargetVelocity() {
        return targetVelocity;
    }

    private double getMetersPerSecToNativeVelUnits(double speedMetersPerSec) {
        return ((speedMetersPerSec * 60.0) * driveGearRatio) / (wheelDiameter * Math.PI);
    }
}
