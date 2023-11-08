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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.subsystems.Drive;
import frc.robot.Constants.DriveConstants;

public class MoveToPose extends CommandBase {
  /** Creates a new MoveToPose. */
  private Pose2d goal;
  private Drive drivetrain;
  private Constraints linearConstraints;
  private Constraints angularConstraints;
  private final Timer timer = new Timer();

  public MoveToPose(Pose2d goal, Drive drivetrain, Constraints linearConstraints, Constraints angularConstraints) {
    this.drivetrain = drivetrain;
    this.goal = goal;
    this.linearConstraints = linearConstraints;
    this.angularConstraints = angularConstraints;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    DriveConstants.BREAKER_HOLONOMIC_DRIVE_CONTROLLER.reset(drivetrain.getOdometryPoseMeters()/*, drivetrain.getFieldRelativeChassisSpeeds()*/);
    BreakerLog.getInstance().logEvent(String.format("MoveToPose command instance STARTED (tgt: %s)", goal.toString()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //ChassisSpeeds targetSpeeds = DriveConstants.BREAKER_HOLONOMIC_DRIVE_CONTROLLER.calculate(goal, drivetrain.getOdometryPoseMeters(), linearConstraints, angularConstraints);
    ChassisSpeeds targetSpeeds = DriveConstants.BREAKER_HOLONOMIC_DRIVE_CONTROLLER.calculate(drivetrain.getOdometryPoseMeters(), goal, 0.1);
    System.out.println(targetSpeeds);
    drivetrain.applyRequest(DriveConstants.MOVE_TO_POSE_REQUEST.withChassisSpeeds(targetSpeeds));
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
      BreakerLog.getInstance().logEvent(String.format("MoveToPose command instance FAILED, command timed out or was cancled (tgt: %s)", goal.toString()));
    } else {
      BreakerLog.getInstance().logEvent(String.format("MoveToPose command instance SUCSEEDED, command reached tgt pose (tgt: %s)", goal.toString()));
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
