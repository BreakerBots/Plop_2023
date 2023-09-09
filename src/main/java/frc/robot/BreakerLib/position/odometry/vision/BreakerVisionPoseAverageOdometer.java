// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.vision.BreakerGenericFiducialTarget;
import frc.robot.BreakerLib.devices.vision.photon.BreakerFiducialPhotonTarget;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.util.math.averages.BreakerAverage;

/** Add your docs here. */
public class BreakerVisionPoseAverageOdometer extends SubsystemBase implements BreakerGenericVisionOdometer {

    private BreakerGenericFiducialTarget[] fiducialTargets;
    private Pose2d curPose;
    private Transform2d offset;
    private BreakerAverage xAvg, yAvg, thetaAvg, timestampAvg;
    private double timestamp;
    public BreakerVisionPoseAverageOdometer(BreakerGenericFiducialTarget... fiducialTargets) {
        curPose = new Pose2d();
        offset = new Transform2d();
        xAvg = new BreakerAverage();
        yAvg = new BreakerAverage();
        thetaAvg = new BreakerAverage();
    }

    @Override
    public double getDataTimestamp() {
        return timestamp;
    }

    @Override
    public boolean isAnyTargetVisable() {
        for (BreakerGenericFiducialTarget fiducialTarget: fiducialTargets) {
            if (fiducialTarget.isAssignedTargetVisible()) {
                return true;
            }
        }
        return false;
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
        for (BreakerGenericFiducialTarget fiducialTarget: fiducialTargets) {
            if (fiducialTarget.isAssignedTargetVisible()) {
                Pose2d tgtPose = fiducialTarget.getRobotPose();
                xAvg.addValue(tgtPose.getX());
                yAvg.addValue(tgtPose.getY());
                thetaAvg.addValue(tgtPose.getRotation().getRadians());
                timestampAvg.addValue(fiducialTarget.getTargetDataTimestamp());
            }
        }
        Pose2d pos = new Pose2d(xAvg.getAverage(), yAvg.getAverage(), Rotation2d.fromRadians(thetaAvg.getAverage()));
        timestamp = timestampAvg.getAverage();
        curPose = pos.transformBy(offset);
        xAvg.clear();
        yAvg.clear();
        thetaAvg.clear();
        timestampAvg.clear();
    }

    @Override
    public void periodic() {
        updatePose();
    }
    
}
