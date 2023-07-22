// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.motorgroups;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestableBase;

/** Add your docs here. */
public abstract class BreakerDiffDriveMotorGroup extends BreakerSelfTestableBase {
    private MotorControllerGroup motorGroup;
    private DoubleSupplier rotorPositionRotationsSupplier, rotorVelocityRotationsPerSecondSupplier;
    protected BreakerDiffDriveMotorGroup(boolean invert, DoubleSupplier rotorPositionRotationsSupplier, DoubleSupplier rotorVelocityRotationsPerSecondSupplier, MotorController... motors) {
        motorGroup = new MotorControllerGroup(motors);
        motorGroup.setInverted(invert);

    }

    public abstract void setBrakeMode(boolean isEnabled);

    public abstract void setRotorPosition(double newPosition);

    public void resetRotorPosition() {
        setRotorPosition(0.0);
    }

    public void set(double dutyCycle) {
        motorGroup.set(dutyCycle);
    }

    public void setVoltage(double voltage) {
        motorGroup.setVoltage(voltage);
    }

    public double getOutputDutyCycle() {
        return motorGroup.get();
    }

    public double getOutputVoltage() {
        return motorGroup.get() * RobotController.getBatteryVoltage();
    }
  
    public double getRotorPosition() {
        return (isInverted() ? -1 : 1) * rotorPositionRotationsSupplier.getAsDouble();
    }

    public double getRotorVelocity() {
        return (isInverted() ? -1 : 1) * rotorVelocityRotationsPerSecondSupplier.getAsDouble();
    }

    public boolean isInverted() {
        return motorGroup.getInverted();
    }
}
