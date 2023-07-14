// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.util.math.BreakerMath;

public class BreakerSwerveOdometer extends SubsystemBase implements BreakerGenericOdometer {
    private SwerveDriveOdometry odometry;
    private BreakerSwerveDrive drivetrain;
    private BreakerMovementState2d prevMovementState = new BreakerMovementState2d(),
      curMovementState = new BreakerMovementState2d();
    private double prevOdometryUpdateTimestamp = 0;
    public BreakerSwerveOdometer(BreakerSwerveDrive drivetrain, Pose2d initialPose) {
        this.drivetrain = drivetrain;
        odometry = new SwerveDriveOdometry(drivetrain.getKinematics(), drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getSwerveModulePositions(), initialPose);
    }

    public BreakerSwerveOdometer(BreakerSwerveDrive drivetrain) {
       this(drivetrain, new Pose2d());
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
      odometry.resetPosition(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getSwerveModulePositions(), newPose);
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
      return odometry.getPoseMeters();
    }

    @Override
    public BreakerMovementState2d getMovementState() {
      return curMovementState;
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
      return drivetrain.getKinematics().toChassisSpeeds(drivetrain.getSwerveModuleStates());
    }
  
    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
      return BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), drivetrain.getOdometryPoseMeters().getRotation());
    }

    private void calculateMovementState(double timeToLastUpdateMiliseconds) {
        curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(getOdometryPoseMeters(),
            getFieldRelativeChassisSpeeds(), timeToLastUpdateMiliseconds, prevMovementState);
        prevMovementState = curMovementState;
    }

    @Override
    public void periodic() {
        odometry.update(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getSwerveModulePositions());
        calculateMovementState((Timer.getFPGATimestamp() - prevOdometryUpdateTimestamp) * 1000);
        prevOdometryUpdateTimestamp = Timer.getFPGATimestamp();
    }
    
  }
