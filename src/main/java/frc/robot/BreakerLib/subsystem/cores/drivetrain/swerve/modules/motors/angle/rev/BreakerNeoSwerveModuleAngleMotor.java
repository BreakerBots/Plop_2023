// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.rev;

import java.util.Objects;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveNeoSparkDutyCycleEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerSwerveAzimuthControler;
//import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveModulePIDConfig;
import frc.robot.BreakerLib.util.factory.BreakerLegacyCANCoderFactory;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;
import frc.robot.BreakerLib.util.vendorutil.BreakerREVUtil;

/** Add your docs here. */
public class BreakerNeoSwerveModuleAngleMotor extends BreakerGenericSwerveModuleAngleMotor {
    private CANSparkMax motor;
    private BreakerSwerveAzimuthEncoder encoder;
    private Rotation2d targetAngle;
    private BreakerSwerveAzimuthControler azimuthControler;
    public BreakerNeoSwerveModuleAngleMotor(CANSparkMax motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetDegrees, int supplyCurrentLimit, boolean isMotorInverted,  BreakerSwerveMotorPIDConfig pidConfig) {
        this.motor = motor;
        this.encoder = encoder;
        
        deviceName = "NEO_Swerve_Angle_Motor_(" + motor.getDeviceId() + ")";

        motor.setInverted(isMotorInverted);
        encoder.config(false, encoderAbsoluteAngleOffsetDegrees);
        azimuthControler = null;
        if (encoder.getBaseEncoderType() == BreakerSwerveNeoSparkDutyCycleEncoder.class) {
            SparkMaxPIDController sparkPID = motor.getPIDController();
            sparkPID.setP(pidConfig.kP);
            sparkPID.setI(pidConfig.kI);
            sparkPID.setD(pidConfig.kD);
            sparkPID.setPositionPIDWrappingEnabled(true);
            sparkPID.setPositionPIDWrappingMaxInput(0.5);
            sparkPID.setPositionPIDWrappingMinInput(-0.5);
            sparkPID.setFeedbackDevice(motor.getEncoder());
            azimuthControler = new BreakerSwerveAzimuthControler((Rotation2d tgt) -> {sparkPID.setReference(tgt.getRotations(), ControlType.kPosition);} );
        }

        if (Objects.isNull(azimuthControler)) {
            azimuthControler = new BreakerSwerveAzimuthControler(motor, encoder, pidConfig);
        }

        BreakerREVUtil.checkError(motor.enableVoltageCompensation(12.0), "Failed to config " + deviceName + " voltage compensation");
        BreakerREVUtil.checkError(motor.setSmartCurrentLimit(supplyCurrentLimit),  "Failed to config " + deviceName + " smart current limit");
        targetAngle = new Rotation2d();
    }

    @Override
    public void setTargetAngle(Rotation2d targetAngle) {
        azimuthControler.setTargetAngle(targetAngle);
        this.targetAngle = targetAngle;
        
    }

    @Override
    public double getAbsoluteAngle() {
        return encoder.getAbsolute();
    }

    @Override
    public double getRelativeAngle() {
        return encoder.getRelative();
    }
    @Override
    public void setBrakeMode(boolean isEnabled) {
        motor.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        Pair<DeviceHealth, String> motorPair = BreakerREVUtil.getSparkMaxHealthAndFaults(motor.getFaults());
        Pair<DeviceHealth, String> encoderPair = encoder.getFaultData();
        if (encoderPair.getFirst() != DeviceHealth.NOMINAL || encoderPair.getFirst() != DeviceHealth.INOPERABLE) {
            health = DeviceHealth.INOPERABLE;
            if (motorPair.getFirst() != DeviceHealth.NOMINAL) {
                faultStr += " ANGLE_MOTOR_FAULTS : " + motorPair.getSecond();
            }
            if (encoderPair.getFirst() != DeviceHealth.NOMINAL) {
                faultStr += " ENCODER_FAULTS : " + encoderPair.getSecond();
            }
        }
    }

}
