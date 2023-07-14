// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerSwerveAzimuthControler;
import frc.robot.BreakerLib.util.factory.BreakerLegacyCANCoderFactory;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;

/** Add your docs here. */
@Deprecated
public class BreakerLegacyFalconSwerveModuleAngleMotor extends BreakerGenericSwerveModuleAngleMotor {
    private WPI_TalonFX motor;
    private BreakerSwerveAzimuthEncoder encoder;
    private Rotation2d targetAngle;
    private BreakerSwerveAzimuthControler azimuthControler;
    public BreakerLegacyFalconSwerveModuleAngleMotor(WPI_TalonFX motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetDegrees, double supplyCurrentLimit, boolean isMotorInverted,  BreakerSwerveMotorPIDConfig pidConfig) {
        this.motor = motor;
        this.encoder = encoder;
        encoder.config(false, encoderAbsoluteAngleOffsetDegrees);
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        if (encoder.getBaseEncoderType() == WPI_CANCoder.class) {
            WPI_CANCoder cancoder = (WPI_CANCoder) encoder.getBaseEncoder();
            turnConfig.remoteFilter0.remoteSensorDeviceID = cancoder.getDeviceID();
            turnConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
            turnConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
            turnConfig.slot0.kP = pidConfig.kP;
            turnConfig.slot0.kI = pidConfig.kI;
            turnConfig.slot0.kD =pidConfig.kD;
            turnConfig.slot0.closedLoopPeakOutput = 1.0;
            azimuthControler = new BreakerSwerveAzimuthControler((Rotation2d target) -> {
                double relTgtAng = BreakerMath.absoluteAngleToContinuousRelativeAngleDegrees(getRelativeAngle(),
                Rotation2d.fromDegrees(getAbsoluteAngle()), target);
                motor.set(TalonFXControlMode.Position, BreakerUnits.degreesToCANCoderNativeUnits(relTgtAng));
            });
        } else {
            azimuthControler = new BreakerSwerveAzimuthControler(motor, encoder, pidConfig);
        }
        turnConfig.peakOutputForward = 1.0;
        turnConfig.peakOutputReverse = -1.0;
        turnConfig.voltageCompSaturation = 12.0;
        turnConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, supplyCurrentLimit, encoderAbsoluteAngleOffsetDegrees, supplyCurrentLimit);
        BreakerPhoenix5Util.checkError(motor.configAllSettings(turnConfig),
                " Failed to config swerve module turn motor ");
        motor.selectProfileSlot(0, 0);
        motor.setSensorPhase(true);
        motor.setInverted(isMotorInverted);
        motor.setNeutralMode(NeutralMode.Brake);
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
       motor.setNeutralMode(isEnabled ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        Pair<DeviceHealth, String> motorPair = BreakerPhoenix5Util.checkMotorFaultsAndConnection(motor);
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
