// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class SetOdometryToVisionMeasurment extends CommandBase {
  /** Creates a new SetOdometryToVisionMeasurment. */
  private Drive drivetrain;
  private Vision vision;
  private boolean waitForTargets, poseReset = false;
  public SetOdometryToVisionMeasurment(Drive drivetrain, Vision vision, boolean waitForTargets) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.waitForTargets = waitForTargets;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    poseReset = false;
    resetPose();
    if (!waitForTargets && !poseReset) {
      BreakerLog.logEvent("Drivetrain odometry pose reset to current vision measurment FAILED, no targets visable, instant fallthrough true");
    }
  }

  @Override
  public void execute() {
    if (waitForTargets && !poseReset) {
      resetPose();
    }
  }

  private void resetPose() {
    if (vision.isAnyTargetVisable()) {
      drivetrain.setAbsoluteOdometryPosition(vision.getOdometryPoseMeters());
      BreakerLog.logEvent("Drivetrain odometry pose reset to current vision measurment");
      poseReset = true;
    } else {
      poseReset = false;
    }
  }

  @Override
  public boolean isFinished() {
    return !waitForTargets || poseReset;
  }
}
