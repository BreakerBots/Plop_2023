// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;


import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.factory.BreakerLegacyCANCoderFactory;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;

/** Add your docs here. */
@Deprecated
public class BreakerSwerveLegacyCANcoder implements BreakerSwerveAzimuthEncoder {
    private WPI_CANCoder encoder;
    public BreakerSwerveLegacyCANcoder(WPI_CANCoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public double getRelative() {
        return encoder.getPosition();
    }

    @Override
    public double getAbsolute() {
        return encoder.getAbsolutePosition();
    }

    @Override
    public Pair<DeviceHealth, String> getFaultData() {
        return BreakerPhoenix5Util.checkCANCoderFaultsAndConnection(encoder);
    }

    @Override
    public Class<?> getBaseEncoderType() {
        return WPI_CANCoder.class;
    }

    @Override
    public Object getBaseEncoder() {
        return encoder;
    }

    @Override
    public void config(boolean invertEncoder, double offset) {
        BreakerLegacyCANCoderFactory.configExistingCANCoder(encoder, SensorInitializationStrategy.BootToAbsolutePosition,
        AbsoluteSensorRange.Signed_PlusMinus180, offset, invertEncoder);
    }

}