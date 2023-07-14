// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/**
 * A duty cycle encoder connected to a spark max using a NEO motor
 */
public class BreakerSwerveNeoSparkDutyCycleEncoder extends SubsystemBase implements BreakerSwerveAzimuthEncoder {

    private AbsoluteEncoder dutyCycleEncoder;
    private RelativeEncoder internalEncoder;
    private CANSparkMax spark;


    /**
     * Creates a REV absolute encoder connected to a SPARK MAX.
     * 
     * @param spark SPARK MAX the encoder is connected to.
     */
    public BreakerSwerveNeoSparkDutyCycleEncoder(CANSparkMax spark, int averageSamplingBitDepth) {
        this.spark = spark;
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        dutyCycleEncoder = spark.getAbsoluteEncoder(Type.kDutyCycle);
        dutyCycleEncoder.setPositionConversionFactor(360);
        dutyCycleEncoder.setAverageDepth(averageSamplingBitDepth);
        internalEncoder = spark.getEncoder();
    }

    @Override
    public double getRelative() {
        return internalEncoder.getPosition();
    }

    @Override
    public double getAbsolute() {
        return internalEncoder.getPosition();
    }

    @Override
    public void config(boolean invertEncoder, double offset) {
        dutyCycleEncoder.setInverted(invertEncoder);
        dutyCycleEncoder.setZeroOffset(offset);
        internalEncoder.setPosition(dutyCycleEncoder.getPosition());
    }

    @Override
    public Pair<DeviceHealth, String> getFaultData() {
        return new Pair<DeviceHealth,String>(DeviceHealth.NOMINAL, "");
    }

    @Override
    public Class<?> getBaseEncoderType() {
        return BreakerSwerveNeoSparkDutyCycleEncoder.class;
    }

    @Override
    public Object getBaseEncoder() {
        return internalEncoder;
    }

    @Override
    public void periodic() {
        if (internalEncoder.getVelocity() < 0.002)  {
            internalEncoder.setPosition(dutyCycleEncoder.getPosition());
        }
    }
}
