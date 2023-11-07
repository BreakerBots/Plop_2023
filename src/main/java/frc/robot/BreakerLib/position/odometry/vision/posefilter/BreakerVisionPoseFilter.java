// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.vision.posefilter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.devices.vision.BreakerGenericFiducialTarget;
import frc.robot.BreakerLib.devices.vision.photon.BreakerFiducialPhotonTarget;
import frc.robot.BreakerLib.util.math.averages.BreakerAverage;
import frc.robot.BreakerLib.util.math.averages.BreakerWeightedAverage;

/** Weighted average for vision-based pose predictions. */
public class BreakerVisionPoseFilter {
    private BreakerGenericFiducialTarget[] positioningTargets;
    private BreakerWeightedAverage avgTimestamp;
    private double trustCoef, maxUncertainty, distanceScailFactor, maxDistance;
    private boolean usesDistanceScaleing = false;

    /**
     * Fiters all predcted poses from visable Fiducial targets through a weaighted
     * average (Without distance scaleing). Weaights calculated by:
     * trustCoef ^ ((-trustCoef) * poseUncertanty)
     * 
     * https://www.desmos.com/calculator/e0nejygfri
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
        avgTimestamp = new BreakerWeightedAverage();
        usesDistanceScaleing = false;
    }

     /**
     * Fiters all predcted poses from visable Fiducial targets through a weaighted
     * average (With distance scaleing). Weaights calculated by:
     * (trustCoef ^ ((-trustCoef) * poseUncertanty)) - (((targetDistance/maxDistance)^2) * distanceScailFactor)
     * 
     * https://www.desmos.com/calculator/gjvf2porer
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
        avgTimestamp = new BreakerWeightedAverage();
        usesDistanceScaleing = true;
    }

    

    /** @return Robot pose with weighted average applied. */
    public Pose3d getFilteredRobotPose3d() {
        BreakerWeightedAverage xAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage yAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage zAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage wAngAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage xAngAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage yAngAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage zAngAverage = new BreakerWeightedAverage();
        avgTimestamp.clear();
        for (int i = 0; i < positioningTargets.length; i++) {
            BreakerGenericFiducialTarget tgt = positioningTargets[i];
            if (tgt.isAssignedTargetVisible()) {
                if (tgt.getPoseAmbiguity() <= maxUncertainty && (usesDistanceScaleing ? tgt.getDistance() <= maxDistance : true)) {
                    double weight = getWeight(tgt);
                    Pose3d pose = tgt.getRobotPose3d();
                    xAverage.addValue(pose.getX(), weight);
                    yAverage.addValue(pose.getY(), weight);
                    zAverage.addValue(pose.getZ(), weight);
                    wAngAverage.addValue(pose.getRotation().getQuaternion().getW(), weight);
                    xAngAverage.addValue(pose.getRotation().getQuaternion().getX(), weight);
                    yAngAverage.addValue(pose.getRotation().getQuaternion().getY(), weight);
                    zAngAverage.addValue(pose.getRotation().getQuaternion().getZ(), weight);
                    avgTimestamp.addValue(tgt.getTargetDataTimestamp(), weight);
                }
            }
        }

        double x = xAverage.getAverage();
        double y = yAverage.getAverage();
        double z = zAverage.getAverage();

        double qW = wAngAverage.getAverage();
        double qX = xAngAverage.getAverage();
        double qY = yAngAverage.getAverage();
        double qz = zAngAverage.getAverage();

        return new Pose3d(x, y, z, new Rotation3d(new Quaternion(qW, qX, qY, qz)));
    }

    public Pose2d getFilteredRobotPose() {
        // note, does not simply convert from 3d pose to conserve CPU time
        BreakerWeightedAverage xAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage yAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage xAngAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage yAngAverage = new BreakerWeightedAverage();
        avgTimestamp.clear();
        for (int i = 0; i < positioningTargets.length; i++) {
            BreakerGenericFiducialTarget tgt = positioningTargets[i];
            if (tgt.isAssignedTargetVisible()) {
                if (tgt.getPoseAmbiguity() <= maxUncertainty && (usesDistanceScaleing ? tgt.getDistance() <= maxDistance : true)) {
                    double weight = getWeight(tgt);
                    Pose2d pose = tgt.getRobotPose();
                    xAverage.addValue(pose.getX(), weight);
                    yAverage.addValue(pose.getY(), weight);
                    xAngAverage.addValue(pose.getRotation().getCos(), weight);
                    yAngAverage.addValue(pose.getRotation().getSin(), weight);
                    avgTimestamp.addValue(tgt.getTargetDataTimestamp(), weight);
                }
            }
        }

        double x = xAverage.getAverage();
        double y = yAverage.getAverage();
        double yAng = yAngAverage.getAverage();
        double xAng = xAngAverage.getAverage();
      
        return new Pose2d(x, y, new Rotation2d(xAng, yAng));
    }

    private double getWeight(BreakerGenericFiducialTarget tgt) {
        double weight = Math.pow(trustCoef, (-trustCoef) * tgt.getPoseAmbiguity());
        if (usesDistanceScaleing) {
            double distFractSq = (tgt.getDistance()/maxDistance) * (tgt.getDistance()/maxDistance);
            weight -= MathUtil.clamp(distFractSq * distanceScailFactor, 0.0, 1.0);
        }
        weight = MathUtil.clamp(weight, 0.0, 1.0);
        return weight;
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
        return avgTimestamp.getAverage();
    }

}
