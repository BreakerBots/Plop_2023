// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint;

import frc.robot.BreakerLib.control.BreakerHolonomicDriveController;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;

/** Add your docs here. */
public class BreakerSwerveWaypointFollowerConfig {
    private BreakerHolonomicDriveController driveController;
    private BreakerSwerveDrive drivetrain;
    private BreakerGenericOdometer odometer;

    public BreakerSwerveWaypointFollowerConfig(BreakerSwerveDrive drivetrain, BreakerHolonomicDriveController driveController) {
        this(drivetrain, drivetrain, driveController);
    }

    public BreakerSwerveWaypointFollowerConfig(BreakerSwerveDrive drivetrain, BreakerGenericOdometer odometer, BreakerHolonomicDriveController driveController) {
        this.driveController = driveController;
        this.drivetrain = drivetrain;
        this.odometer = odometer;
    }

    public BreakerHolonomicDriveController getDriveController() {
        return driveController;
    }

    public BreakerSwerveDrive getDrivetrain() {
        return drivetrain;
    }

    public BreakerGenericOdometer getOdometer() {
        return odometer;
    }
}
