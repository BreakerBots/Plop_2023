// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;

/** Add your docs here. */
public class BreakerSwerveCANcoder implements BreakerSwerveAzimuthEncoder {
    private CANcoder encoder;
    public BreakerSwerveCANcoder(CANcoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public double getRelative() {
        return BaseStatusSignal.getLatencyCompensatedValue(encoder.getPosition(), encoder.getVelocity());
    }

    @Override
    public double getAbsolute() {
        return MathUtil.inputModulus(BaseStatusSignal.getLatencyCompensatedValue(encoder.getAbsolutePosition(), encoder.getVelocity()), -0.5, 0.5);
    }

    @Override
    public Pair<DeviceHealth, String> getFaultData() {
        return BreakerPhoenix6Util.checkCANcoderFaultsAndConnection(encoder);
    }

    @Override
    public Class<?> getBaseEncoderType() {
        return CANcoder.class;
    }

    @Override
    public Object getBaseEncoder() {
        return encoder;
    }

    @Override
    public void config(boolean invertEncoder, double absoluteOffset) {
        BreakerCANCoderFactory.configExistingCANCoder(encoder, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, absoluteOffset, invertEncoder ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive);
    }

}