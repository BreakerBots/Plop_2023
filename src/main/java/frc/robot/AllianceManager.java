// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class AllianceManager {

    public static Optional<Alliance> getAlliance() {
        Alliance alliance = DriverStation.getAlliance();
        if ((DriverStation.isDSAttached() || DriverStation.isFMSAttached()) && alliance != Alliance.Invalid) {
            return Optional.of(DriverStation.getAlliance());
        }
        return Optional.empty();
    }
}
