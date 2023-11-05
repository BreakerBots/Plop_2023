// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.actions;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.commands.drive.MoveToPose;
import frc.robot.subsystems.Drive;

public class NavToPathStart extends ConditionalCommand {
  /** Creates a new NavToPathStart. */
  public NavToPathStart(Drive drivetrain, PathPlannerTrajectory trajectory) {
    super(new InstantCommand(() -> BreakerLog.getInstance().logEvent("NavPathToStart command fellthrough, robot WITHIN TOLERENCE of traj init pose")), new MoveToPose(trajectory.getInitialHolonomicPose(), drivetrain, AutonomousConstants.NAV_TO_TRAJECTORY_START_LINEAR_CONSTRAINTS, AutonomousConstants.NAV_TO_TRAJECTORY_START_ANGULAR_CONSTRAINTS), () -> {return inTolerence(drivetrain, trajectory);});
  }

  private static boolean inTolerence(Drive drivetrain, PathPlannerTrajectory trajectory) {
    return BreakerMath.epsilonEqualsPose2d(trajectory.getInitialHolonomicPose(), drivetrain.getOdometryPoseMeters(), AutonomousConstants.NAV_TO_TRAJECTORY_START_TOLERENCE);
  }
}
