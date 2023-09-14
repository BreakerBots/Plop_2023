// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/**
 * A duty cycle encoder connected to a spark max using a NEO motor
 */
public class BreakerSwerveSparkDutyCycleEncoder implements BreakerSwerveAzimuthEncoder {

    private AbsoluteEncoder dutyCycleEncoder;


    /**
     * Creates a REV absolute encoder connected to a SPARK MAX.
     * 
     * @param spark SPARK MAX the encoder is connected to.
     */
    public BreakerSwerveSparkDutyCycleEncoder(CANSparkMax spark, int averageSamplingBitDepth) {
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        dutyCycleEncoder = spark.getAbsoluteEncoder(Type.kDutyCycle);
        dutyCycleEncoder.setPositionConversionFactor(1.0);
        dutyCycleEncoder.setAverageDepth(averageSamplingBitDepth);
    }

    @Override
    public double getRelative() {
        return dutyCycleEncoder.getPosition();
    }

    @Override
    public double getAbsolute() {
        return dutyCycleEncoder.getPosition();
    }

    @Override
    public void config(boolean invertEncoder, double offset) {
        dutyCycleEncoder.setInverted(invertEncoder);
        dutyCycleEncoder.setZeroOffset(offset);
    }

    @Override
    public Pair<DeviceHealth, String> getFaultData() {
        return new Pair<DeviceHealth,String>(DeviceHealth.NOMINAL, "");
    }

    @Override
    public Class<?> getBaseEncoderType() {
        return BreakerSwerveSparkDutyCycleEncoder.class;
    }

    @Override
    public Object getBaseEncoder() {
        return dutyCycleEncoder;
    }
}
