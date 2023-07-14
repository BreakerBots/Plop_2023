// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionPoseFilterOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;

/** Estimates swerve drive pose based on vision odometry. */
public class BreakerSwerveDriveFusedVisionPoseEstimator
        extends SubsystemBase implements BreakerGenericOdometer {
    private BreakerGenericVisionOdometer vision;
    private BreakerSwerveDrivePoseEstimator poseEstimator;

    public BreakerSwerveDriveFusedVisionPoseEstimator(
            BreakerSwerveDrive drivetrain,
            BreakerGenericVisionOdometer vision,
            double[] stateStanderdDeveation,
            double[] visionStanderdDeveation) {
        this.vision = vision;
        poseEstimator = new BreakerSwerveDrivePoseEstimator(
                drivetrain, vision.getOdometryPoseMeters(),
                stateStanderdDeveation, visionStanderdDeveation);
    }

    public BreakerSwerveDriveFusedVisionPoseEstimator(
            BreakerSwerveDrive drivetrain,
            BreakerGenericVisionOdometer vision,
            Pose2d initialPose,
            double[] stateStandardDeviation,
            double[] visionStandardDeviation) {
        this.vision = vision;
        vision.setOdometryPosition(initialPose);
        poseEstimator = new BreakerSwerveDrivePoseEstimator(
                drivetrain, initialPose,
                stateStandardDeviation, visionStandardDeviation);
    }

    public void changeVisionDevs(double visionStrdDevX, double visionStrdDevY, double visionStrdDevTheta) {
        poseEstimator.changeVisionDevs(visionStrdDevX, visionStrdDevY, visionStrdDevTheta);;
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        vision.setOdometryPosition(newPose);
        poseEstimator.setOdometryPosition(newPose);
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        return poseEstimator.getOdometryPoseMeters();
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        return poseEstimator.getMovementState();
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return poseEstimator.getRobotRelativeChassisSpeeds();
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return poseEstimator.getFieldRelativeChassisSpeeds();
    }

    private void updateOdometry() {
        if (vision.isAnyTargetVisable()) {
            poseEstimator.addVisionMeasurment(vision.getOdometryPoseMeters(), vision.getDataTimestamp());
        }
    }

    @Override
    public void periodic() {
        updateOdometry();
    }
}
