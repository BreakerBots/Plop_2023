// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public interface BreakerSwerveAzimuthEncoder {

    /** @return Relative anglular position in degrees, (180 -> 181) */
    public abstract double getRelative();

    /** @return Absolute anglular position in degrees [-180, 180]. */
    public abstract double getAbsolute();

    /**
     * Configures the encoder.
     * 
     * @param clockwisePositive Sets whether roation that would normaly be read as positive should be negative
     * @param offset            Angle offset in degrees.
     */
    public abstract void config(boolean invertEncoder, double offset);

    public abstract Pair<DeviceHealth, String> getFaultData();

    public abstract Class<?> getBaseEncoderType();

    public abstract Object getBaseEncoder();
}