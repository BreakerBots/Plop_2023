// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.pathplanner;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoPath;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveBase;

/** Add your docs here. */
public class BreakerSwerveAutoRoutine extends BreakerAutoPath  {
    private BreakerSwerveDrive drivetrain;
    private PPHolonomicDriveController driveController;
    private SequentialCommandGroup commandGroup;
    public BreakerSwerveAutoRoutine(BreakerSwerveDriveBase drivetrain, HashMap<String, Command> eventMap, String pathName) {
        this(drivetrain, drivetrain.getConfig().getDriveController(), eventMap, pathName);
    }

    public BreakerSwerveAutoRoutine(BreakerSwerveDrive drivetrain, PPHolonomicDriveController driveController, HashMap<String, Command> eventMap, String pathName) {
        this.pathName = pathName;
        this.drivetrain = drivetrain;
        this.driveController = driveController;
        this.commandGroup = new SequentialCommandGroup(new Command[0]);
        this.autoPath = commandGroup;

    }

    // public void add(PathPlannerTrajectory trajectory) {
    //     commandGroup.addCommands(new BreakerSwervePathFollower(pathFollowerConfig, trajectory, true));

}
