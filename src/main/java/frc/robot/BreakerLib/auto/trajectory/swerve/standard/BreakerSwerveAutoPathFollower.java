// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve.standard;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.auto.trajectory.BreakerGenericAutoPathFollower;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalEvent;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveMovementPreferences;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveMovementPreferences.SwerveMovementRefrenceFrame;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/**
 * A command that allows a {@link BreakerLegacySwerveDrive} to follow a
 * {@link BreakerTrajectoryPath}
 */
public class BreakerSwerveAutoPathFollower extends CommandBase implements BreakerGenericAutoPathFollower {
  
  private final Timer timer = new Timer();
  private BreakerSwerveAutoPathFollowerConfig config;
  private BreakerTrajectoryPath trajectoryPath;
  private Supplier<Rotation2d> rotationSupplier;
  private ArrayList<BreakerConditionalEvent> remainingEvents;
  private final BreakerSwerveMovementPreferences movementPreferences;

  /**
   * Creates a new {@link BreakerSwerveAutoPathFollower} that follows the given
   * {@link BreakerTrajectoryPath}
   * <p>
   * NOTE: Robot's rotation setpoint defaults to 0 degrees
   * 
   * @param config         The {@link BreakerSwerveAutoPathFollowerConfig}
   *                       instance that defines this
   *                       {@link BreakerSwerveAutoPathFollower} instances base
   *                       setup
   * @param trajectoryPath The {@link BreakerTrajectoryPath} that this
   *                       {@link BreakerSwerveAutoPathFollower} will follow
   */
  public BreakerSwerveAutoPathFollower(BreakerSwerveAutoPathFollowerConfig config,
      BreakerTrajectoryPath trajectoryPath) {
    addRequirements(config.getDrivetrain());
    this.config = config;
    this.trajectoryPath = trajectoryPath;
    rotationSupplier = () -> (trajectoryPath.getBaseTrajectory().sample(timer.get()).poseMeters.getRotation());
    remainingEvents = new ArrayList<>(trajectoryPath.getAttachedConditionalEvents());
    movementPreferences = new BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITHOUT_OFFSET, SlowModeValue.DISABLED);
  }

  /**
   * Creates a new {@link BreakerSwerveAutoPathFollower} that follows the given
   * {@link BreakerTrajectoryPath}
   * 
   * @param config           The {@link BreakerSwerveAutoPathFollowerConfig}
   *                         instance that defines this
   *                         {@link BreakerSwerveAutoPathFollower} instances base
   *                         setup
   * @param trajectoryPath   The {@link BreakerTrajectoryPath} that this
   *                         {@link BreakerSwerveAutoPathFollower} will follow
   * @param rotationSupplier The {@link BreakerGenericSwerveRotationSupplier} that
   *                         returns this path follower's rotation setpoint.
   */
  public BreakerSwerveAutoPathFollower(BreakerSwerveAutoPathFollowerConfig config, BreakerTrajectoryPath trajectoryPath,
      Supplier<Rotation2d> rotationSupplier) {
    addRequirements(config.getDrivetrain());
    this.config = config;
    this.trajectoryPath = trajectoryPath;
    this.rotationSupplier = rotationSupplier;
    remainingEvents = new ArrayList<>();
    movementPreferences = new BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITHOUT_OFFSET, SlowModeValue.DISABLED);
  }

  @Override
  public void initialize() {
    BreakerLog.logBreakerLibEvent("A new BreakerSwerveAutoPathFollower instance has started");
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double curTime = timer.get();
    State desiredState = trajectoryPath.getBaseTrajectory().sample(curTime);

    ChassisSpeeds targetChassisSpeeds = config.getDriveController()
        .calculate(config.getOdometer().getOdometryPoseMeters(), desiredState, rotationSupplier.get());

    config.getDrivetrain().move(targetChassisSpeeds, movementPreferences);

    if (remainingEvents.size() > 0) {
      Iterator<BreakerConditionalEvent> iterator = remainingEvents.iterator();
      while (iterator.hasNext()) {
        BreakerConditionalEvent ev = iterator.next();
        if (ev.checkCondition(timer.get(), config.getOdometer().getOdometryPoseMeters())) {
          ev.getBaseCommand().schedule();
          iterator.remove();
        }
      }
    }

  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    if (trajectoryPath.stopAtEnd()) {
      config.getDrivetrain().stop();
    }
    if (interrupted) {
      BreakerLog.logBreakerLibEvent("A BreakerSwerveAutoPathFollower instance was interrupted");
    } else {
      BreakerLog.logBreakerLibEvent("A BreakerSwerveAutoPathFollower instance has ended normaly");
    }

  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectoryPath.getBaseTrajectory().getTotalTimeSeconds());
  }

  @Override
  public BreakerTrajectoryPath getTrajectoryPath() {
    return trajectoryPath;
  }

  @Override
  public double getElapsedTimeSeconds() {
    return timer.get();
  }

  @Override
  public boolean getPathStopsAtEnd() {
    return trajectoryPath.stopAtEnd();
  }

  @Override
  public void attachConditionalEvents(BreakerConditionalEvent... conditionalEvents) {
    for (BreakerConditionalEvent ev : conditionalEvents) {
      remainingEvents.add(ev);
    }
  }
}
