// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.subsystems.OffseasionBotDrive;
import frc.robot.Constants.DriveConstants;

public class MoveToPose extends CommandBase {
  /** Creates a new MoveToPose. */
  private Pose2d goal;
  private OffseasionBotDrive drivetrain;
  private double maxLinearVel;
  private final Timer timer = new Timer();

  public MoveToPose(Pose2d goal, double maxLinearVel, OffseasionBotDrive drivetrain) {
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds targetSpeeds = DriveConstants.BREAKER_HOLONOMIC_DRIVE_CONTROLLER.calculate(goal, drivetrain.getOdometryPoseMeters(), maxLinearVel);
    drivetrain.move(targetSpeeds, DriveConstants.MOVE_TO_POSE_MOVEMENT_PREFERENCES);
    if (timer.hasElapsed(DriveConstants.MOVE_TO_POSE_TIMEOUT_SEC)) {
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    drivetrain.stop();
    if (interrupted) {
      BreakerLog.logEvent("MoveToPose command instance FAILED, command timed out or was cancled");
    } else {
      BreakerLog.logEvent("MoveToPose command instance SUCSEEDED, command reached tgt pose");
    }
  }

  public boolean atSetpoint() {
    boolean atPose = BreakerMath.epsilonEqualsPose2d(goal, drivetrain.getOdometryPoseMeters(), DriveConstants.BHDC_POSE_TOLERENCE);
    boolean atVel = BreakerMath.epsilonEqualsChassisSpeeds(new ChassisSpeeds(), drivetrain.getFieldRelativeChassisSpeeds(), DriveConstants.BHDC_VELOCITY_TOLERENCE);
    return atPose && atVel;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atSetpoint();
  }
}
