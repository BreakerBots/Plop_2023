// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.devices.vision.BreakerGenericFiducialTarget;
import frc.robot.BreakerLib.devices.vision.photon.BreakerFiducialPhotonTarget;
import frc.robot.BreakerLib.util.math.averages.BreakerAverage;
import frc.robot.BreakerLib.util.math.averages.BreakerWeightedAverage;

/** Weighted average for vision-based pose predictions. */
public class BreakerVisionPoseFilter {
    private BreakerGenericFiducialTarget[] positioningTargets;
    private BreakerAverage avgLatency;
    private double trustCoef, maxUncertainty, distanceScailFactor, maxDistance;
    private boolean usesDistanceScaleing = false;

    /**
     * Fiters all predcted poses from visable Fiducial targets through a weaighted
     * average (Without distance scaleing). Weaights calculated by:
     * trustCoef ^ ((-trustCoef) * poseUncertanty)
     * 
     * @param trustCoef          - Higher values mean more uncertain values are
     *                           trusted less.
     * @param maxUncertainty     - The highest uncertainty value (0-1) that will
     *                           still be considered in the pose calculation.
     * @param positioningTargets - The fiducial targets for positioning.
     */
    public BreakerVisionPoseFilter(double trustCoef, double maxUncertainty,
            BreakerGenericFiducialTarget... positioningTargets) {
        this.positioningTargets = positioningTargets;
        this.trustCoef = MathUtil.clamp(trustCoef, 1, Double.MAX_VALUE);
        this.maxUncertainty = maxUncertainty;
        avgLatency = new BreakerAverage();
        usesDistanceScaleing = false;
    }

     /**
     * Fiters all predcted poses from visable Fiducial targets through a weaighted
     * average (With distance scaleing). Weaights calculated by:
     * (trustCoef ^ ((-trustCoef) * poseUncertanty)) - ((targetDistance/maxDistance) * distanceScailFactor)
     * 
     * @param trustCoef           - Higher values mean more uncertain values are
     *                           trusted less.
     * @param maxUncertainty      - The highest uncertainty value (0-1) that will
     *                           still be considered in the pose calculation.
     * @param positioningTargets  - The fiducial targets for positioning.
     * 
     * @param distanceScailFactor - Scail factor (0-1) applyed to the targets distance from the camera as a percentage of max distance
     * 
     * @param maxDistance - The maximum acceptaible disance of a target from the camera
     */
    public BreakerVisionPoseFilter(
            double trustCoef, double maxUncertainty, 
            double distanceScailFactor, double maxDistance,
            BreakerGenericFiducialTarget... positioningTargets
        ) {
        this.positioningTargets = positioningTargets;
        this.distanceScailFactor = MathUtil.clamp(distanceScailFactor, 1, Double.MAX_VALUE);
        this.trustCoef = MathUtil.clamp(trustCoef, 1, Double.MAX_VALUE);
        this.maxDistance = maxDistance;
        this.maxUncertainty = maxUncertainty;
        avgLatency = new BreakerAverage();
        usesDistanceScaleing = true;
    }

    

    /** @return Robot pose with weighted average applied. */
    public Pose3d getFilteredRobotPose3d() {
        BreakerWeightedAverage xAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage yAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage zAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage yAngAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage pAngAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage rAngAverage = new BreakerWeightedAverage();
        avgLatency.clear();
        for (int i = 0; i < positioningTargets.length; i++) {
            BreakerGenericFiducialTarget tgt = positioningTargets[i];
            if (tgt.isAssignedTargetVisible()) {
                if (tgt.getPoseAmbiguity() <= maxUncertainty && (usesDistanceScaleing ? tgt.getDistance() <= maxDistance : true)) {
                    double weight = Math.pow(trustCoef, (-trustCoef) * tgt.getPoseAmbiguity());
                    if (usesDistanceScaleing) {
                        weight -= MathUtil.clamp((tgt.getDistance()/maxDistance) * distanceScailFactor, 0.0, 1.0);
                    }
                    weight = MathUtil.clamp(weight, 0.0, 1.0);
                    Pose3d pose = tgt.getRobotPose3d();
                    xAverage.addValue(pose.getX(), weight);
                    yAverage.addValue(pose.getY(), weight);
                    zAverage.addValue(pose.getZ(), weight);
                    yAngAverage.addValue(pose.getRotation().getZ(), weight);
                    pAngAverage.addValue(pose.getRotation().getY(), weight);
                    rAngAverage.addValue(pose.getRotation().getX(), weight);
                    avgLatency.addValue(tgt.getTargetDataTimestamp());
                }
            }
        }

        double x = xAverage.getAverage();
        double y = yAverage.getAverage();
        double z = zAverage.getAverage();

        double yaw = yAngAverage.getAverage();
        double pitch = pAngAverage.getAverage();
        double roll = rAngAverage.getAverage();

        return new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw));
    }

    public Pose2d getFilteredRobotPose() {
        // note, does not simply convert from 3d pose to conserve CPU time
        BreakerWeightedAverage xAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage yAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage yAngAverage = new BreakerWeightedAverage();
        avgLatency.clear();
        for (int i = 0; i < positioningTargets.length; i++) {
            BreakerGenericFiducialTarget tgt = positioningTargets[i];
            if (tgt.isAssignedTargetVisible()) {
                if (tgt.getPoseAmbiguity() <= maxUncertainty && (usesDistanceScaleing ? tgt.getDistance() <= maxDistance : true)) {
                    double weight = Math.pow(trustCoef, (-trustCoef) * tgt.getPoseAmbiguity());
                    if (usesDistanceScaleing) {
                        weight -= MathUtil.clamp((tgt.getDistance()/maxDistance) * distanceScailFactor, 0.0, 1.0);
                    }
                    weight = MathUtil.clamp(weight, 0.0, 1.0);
                    Pose2d pose = tgt.getRobotPose();
                    xAverage.addValue(pose.getX(), weight);
                    yAverage.addValue(pose.getY(), weight);
                    yAngAverage.addValue(pose.getRotation().getRadians(), weight);
                    avgLatency.addValue(tgt.getTargetDataTimestamp());
                }
            }
        }

        double x = xAverage.getAverage();
        double y = yAverage.getAverage();
        double yaw = yAngAverage.getAverage();
      
        return new Pose2d(x, y, new Rotation2d(yaw));
    }

    public boolean isAnyTargetVisable() {
        for (BreakerGenericFiducialTarget tgt: positioningTargets) {
            if (tgt.isAssignedTargetVisible()) {
                return true;
            }
        }
        return false;
    }

    public double getDataTimestamp() {
        return avgLatency.getAverage();
    }

}
