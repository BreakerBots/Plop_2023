// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;

/** Fiducial marker target. Use if using AprilTags w/ 3D calibrated camera. */
public interface BreakerGenericFiducialTarget { 

    public abstract AprilTag getBaseApriltag();

    /** @return Timestamp of last target found. */
    public abstract double getLastTargetFoundTimestamp();

    /** @return Latency of target data in seconds. */
    public abstract double getTargetDataAge();

    /** @return The timestamp of the vision data */
    public default double getTargetDataTimestamp() {
        return Timer.getFPGATimestamp() - getTargetDataAge();
    }

    /** @return 3d pose of camera. */
    public Pose3d getCameraPose3d();

    /** @return 2d pose of camera. */
    public default Pose2d getCameraPose() {
        return getCameraPose3d().toPose2d();
    }

    /** @return 3d pose of robot from target data. */
    public Pose3d getRobotPose3d();

    /** @return 2d pose of robot from target data. */
    public default Pose2d getRobotPose() {
        return getRobotPose3d().toPose2d();
    }

    /** @return Assigned target camera relative yaw */
    public double getYaw();

    /** @return Assigned target camera relative pitch */
    public double getPitch();
 
    /** @return Target's yaw relative to robot's zero point. */
    public double getRobotRelativeYaw();

    /** @return Target's pitch relative to robot's zero point. */
    public double getRobotRelativePitch();

    /** @return Assigned target skew. */
    public double getSkew();

    /** @return Assigned target area */
    public double getArea();

    // /** @return List of target corner coordinates. */
    // public List<TargetCorner> getTargetCorners();

    /** @return If assigned target has been found at any point during operation */
    public boolean getAssignedTargetFound();

    /** @return If assigned target was found in the most recent cycle */
    public boolean isAssignedTargetVisible();

    /** @return Distance from camera to target along the floor. */
    public double getDistance2D();

    /** @return 3D Euclidian disance between the camera and the target. */
    public double getDistance();

    /** @return ID of fiducial target */
    public int getFiducialID();

    /** @return Ambiguity of pose, from 0 to 1. 0 = most accurate, 1 = least accurate. Anything above 0.2 is likely inaccurate. */
    public double getPoseAmbiguity();
}
