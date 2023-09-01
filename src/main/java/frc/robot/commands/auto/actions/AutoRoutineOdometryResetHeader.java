// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.actions;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class AutoRoutineOdometryResetHeader extends CommandBase {
  /** Creates a new AutoRoutineOdometryResetHeader. */
  private Drive drivetrain;
  private Vision vision;
  private PathPlannerTrajectory trajectory;
  private double waitForVisionTargetTimeout;
  private final Timer timer = new Timer();
  private boolean hasSetPose;
  public AutoRoutineOdometryResetHeader(Drive drivetrain, Vision vision, PathPlannerTrajectory trajectory, double waitForVisionTargetTimeout) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.trajectory = trajectory;
    this.waitForVisionTargetTimeout = waitForVisionTargetTimeout;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    hasSetPose = false;
    if (waitForVisionTargetTimeout == 0.0) {
      visionSet();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hasSetPose) {
      if (timer.hasElapsed(waitForVisionTargetTimeout) || waitForVisionTargetTimeout == 0.0) {
        drivetrain.setAbsoluteOdometryPosition(trajectory.getInitialHolonomicPose());
        hasSetPose = true;
        BreakerLog.logEvent("Robot odometry position reset trajectory initial state, no vision targets visiable");
      } else {
        visionSet();
      }
    }
  }

  private void visionSet() {
    if (vision.isAnyTargetVisable()) {
      drivetrain.setAbsoluteOdometryPosition(vision.getOdometryPoseMeters());
      hasSetPose = true;
      BreakerLog.logEvent("Robot odometry position reset to vision measurment");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasSetPose;
  }
}
