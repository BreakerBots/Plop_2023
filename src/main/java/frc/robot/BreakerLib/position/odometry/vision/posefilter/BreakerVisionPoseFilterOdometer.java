// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.vision.posefilter;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;

/** Add your docs here. */
public class BreakerVisionPoseFilterOdometer extends SubsystemBase implements BreakerGenericVisionOdometer {
    private BreakerVisionPoseFilter visionPoseFilter;
    private Pose2d curPose;
    private Transform2d offset;
    public BreakerVisionPoseFilterOdometer(BreakerVisionPoseFilter visionPoseFilter) {
        this.visionPoseFilter = visionPoseFilter;
        curPose = new Pose2d();
        offset = new Transform2d();
    }

    @Override
    public double getDataTimestamp() {
        return visionPoseFilter.getDataTimestamp();
    }

    @Override
    public boolean isAnyTargetVisable() {
        return visionPoseFilter.isAnyTargetVisable() && !Double.isNaN(curPose.getX()) && !Double.isNaN(curPose.getY()) && !Double.isNaN(curPose.getRotation().getRadians());
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        offset = curPose.minus(newPose);
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        return curPose;
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        return new BreakerMovementState2d();
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        // TODO Auto-generated method stub
        return new ChassisSpeeds();
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        // TODO Auto-generated method stub
        return new ChassisSpeeds();
    }

    private void updateChassisSpeeds() {}

    private void updatePose() {
        Pose2d pos = visionPoseFilter.getFilteredRobotPose();
        if (!Double.isNaN(pos.getX()) && !Double.isNaN(pos.getY()) && !Double.isNaN(pos.getRotation().getRadians()))
        curPose = pos.transformBy(offset);
    }

    @Override
    public void periodic() {
        if (visionPoseFilter.isAnyTargetVisable()) {
            updatePose();
        }
    }
}
