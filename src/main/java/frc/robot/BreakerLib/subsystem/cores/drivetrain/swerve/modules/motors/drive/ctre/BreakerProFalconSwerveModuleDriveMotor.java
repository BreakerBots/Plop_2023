// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.BreakerGenericSwerveModuleDriveMotor;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;

/** Add your docs here. */
public class BreakerProFalconSwerveModuleDriveMotor extends BreakerGenericSwerveModuleDriveMotor {
    private TalonFX motor;
    private double driveGearRatio, wheelDiameter, targetVelocity;
    private final VelocityDutyCycle velocityDutyCycleRequest;
    private final VelocityVoltage velocityVoltageRequest;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest;
    private final double wheelCircumfrenceMeters;
    private BreakerArbitraryFeedforwardProvider arbFF;
    private final  ProTalonFXControlOutputUnits controlOutputUnits;
    public BreakerProFalconSwerveModuleDriveMotor(TalonFX motor, double driveGearRatio, double wheelDiameter, double supplyCurrentLimit, boolean isMotorInverted, BreakerArbitraryFeedforwardProvider arbFF, BreakerSwerveMotorPIDConfig pidConfig, ProTalonFXControlOutputUnits controlOutputUnits) {
        this.motor = motor;
        this.driveGearRatio = driveGearRatio;
        this.wheelDiameter = wheelDiameter;
        this.arbFF = arbFF;
        wheelCircumfrenceMeters = wheelDiameter*Math.PI;
        targetVelocity = 0.0;
        velocityDutyCycleRequest = new VelocityDutyCycle(0.0, true, 0.0, 1, false);
        velocityVoltageRequest = new VelocityVoltage(0.0, true, 0.0, 1, false);
        velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0, 0.0, 1, false);
        this.controlOutputUnits = controlOutputUnits;
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Feedback.SensorToMechanismRatio = driveGearRatio;
        driveConfig.Slot1.kP = pidConfig.kP;
        driveConfig.Slot1.kI = pidConfig.kI;
        driveConfig.Slot1.kD = pidConfig.kD;
        driveConfig.Slot1.kV = pidConfig.kF;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        driveConfig.CurrentLimits.SupplyCurrentThreshold = supplyCurrentLimit;
        driveConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        BreakerPhoenix6Util.checkStatusCode(motor.getConfigurator().apply(driveConfig),
                " Failed to config swerve module drive motor ");
    
        motor.setInverted(isMotorInverted);
        setMotorVel(0.0, 0.0);
    }

    private void setMotorVel(double vel, double ff) {
        switch (controlOutputUnits) {
            case DUTY_CYCLE:
                motor.setControl(velocityDutyCycleRequest.withVelocity(vel).withFeedForward(ff));
                break;
            case TORQUE_CURRENT:
                motor.setControl(velocityTorqueCurrentRequest.withVelocity(vel).withFeedForward(ff));
                break;
            case VOLTAGE:
            default:
                motor.setControl(velocityVoltageRequest.withVelocity(vel).withFeedForward(ff));
                break;
        }
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        Pair<DeviceHealth, String> pair = BreakerPhoenix6Util.checkMotorFaultsAndConnection(motor);
        health = pair.getFirst();
        if (health != DeviceHealth.NOMINAL) {
            faultStr = " DRIVE_MOTOR_FAULTS : " + pair.getSecond();
        }
    }

    @Override
    public void setTargetVelocity(double targetMetersPerSecond) {
        targetVelocity = targetMetersPerSecond;
        setMotorVel(targetMetersPerSecond / wheelCircumfrenceMeters, arbFF.getArbitraryFeedforwardValue(targetMetersPerSecond));
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity().getValue() * wheelCircumfrenceMeters;
    }

    @Override
    public double getDistance() {
        return BaseStatusSignal.getLatencyCompensatedValue(motor.getPosition(), motor.getVelocity()) * wheelCircumfrenceMeters;
    }

    @Override
    public void resetDistance() {
        BreakerPhoenix6Util.checkStatusCode(motor.setRotorPosition(0),
                " Failed to reset swerve module rive motor position ");
        ;
    }

    @Override
    public void setBrakeMode(boolean isEnabled) {
        BreakerPhoenix6Util.setBrakeMode(motor, isEnabled);
    }

    @Override
    public double getTargetVelocity() {
        return targetVelocity;
    }

    @Override
    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValue();
    }

    @Override
    public double getMotorOutput() {
        return motor.getClosedLoopOutput().getValue();
    }

    public enum ProTalonFXControlOutputUnits {
        VOLTAGE,
        DUTY_CYCLE,
        TORQUE_CURRENT
    }
}
