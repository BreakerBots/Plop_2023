// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.ctre;

import java.net.CacheRequest;
import java.util.Objects;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerSwerveAzimuthControler;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;

/** Add your docs here. */
public class BreakerProFalconSwerveModuleAngleMotor extends BreakerGenericSwerveModuleAngleMotor {
    private TalonFX motor;
    private BreakerSwerveAzimuthEncoder encoder;
    private Rotation2d targetAngle;
    private BreakerSwerveAzimuthControler azimuthControler;
    private final PositionDutyCycle positionRequest;
    public BreakerProFalconSwerveModuleAngleMotor(TalonFX motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetDegrees, double azimuthGearRatio, double supplyCurrentLimit, boolean isMotorInverted,  BreakerSwerveMotorPIDConfig pidConfig) {
        this.motor = motor;
        this.encoder = encoder;
        positionRequest = new PositionDutyCycle(0.0, true, 0.0, 0, false);

        encoder.config(false, encoderAbsoluteAngleOffsetDegrees);
        azimuthControler = null;
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        if (encoder.getBaseEncoderType() == CANcoder.class) {
            CANcoder cancoder = (CANcoder) encoder.getBaseEncoder();
            if (cancoder.getCANBus() == motor.getCANBus()) {
                turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                turnConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
                turnConfig.Feedback.RotorToSensorRatio = azimuthGearRatio;
                turnConfig.Feedback.SensorToMechanismRatio = 1.0;
                turnConfig.Slot0.kP = pidConfig.kP;
                turnConfig.Slot0.kI = pidConfig.kI;
                turnConfig.Slot0.kD = pidConfig.kD;
                turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
                azimuthControler = new BreakerSwerveAzimuthControler((Rotation2d target) -> {motor.setControl(positionRequest.withPosition(target.getRotations()));});
            }
        }
        if (Objects.isNull(azimuthControler)) {
            azimuthControler = new BreakerSwerveAzimuthControler(motor, encoder, pidConfig);
        }
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        turnConfig.CurrentLimits.SupplyCurrentThreshold = supplyCurrentLimit;
        turnConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        BreakerPhoenix6Util.checkStatusCode(motor.getConfigurator().apply(turnConfig),
                " Failed to config swerve module drive motor ");
    
        motor.setInverted(isMotorInverted);
        azimuthControler.setTargetAngle(new Rotation2d());
        targetAngle = new Rotation2d();
        deviceName = "TalonFX_Swerve_Angle_Motor_(" + motor.getDeviceID() + ")";
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
        BreakerPhoenix6Util.setBrakeMode(motor, isEnabled);
    }

    @Override
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        Pair<DeviceHealth, String> motorPair = BreakerPhoenix6Util.checkMotorFaultsAndConnection(motor);
        Pair<DeviceHealth, String> encoderPair = encoder.getFaultData();
        if (motorPair.getFirst() != DeviceHealth.NOMINAL || encoderPair.getFirst() != DeviceHealth.NOMINAL) {
            if (motorPair.getFirst() != DeviceHealth.NOMINAL) {
                faultStr += " ANGLE_MOTOR_FAULTS : " + motorPair.getSecond();
            }
            if (encoderPair.getFirst() != DeviceHealth.NOMINAL) {
                faultStr += " ENCODER_FAULTS : " + encoderPair.getSecond();
            }
        }
    }

}
