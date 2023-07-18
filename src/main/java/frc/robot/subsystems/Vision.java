// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
import frc.robot.BreakerLib.devices.vision.photon.BreakerVision;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionPoseFilterOdometer;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase implements BreakerGenericVisionOdometer {
  /** Creates a new Vision. */
  private BreakerPhotonCamera frontCam, leftCam, rightCam, backCam;
  private BreakerVision vision;
  public Vision() {
    configApriltags();
    frontCam = new BreakerPhotonCamera(VisionConstants.FRONT_CAMERA_NAME, VisionConstants.FRONT_CAMERA_POSE);
    leftCam  = new BreakerPhotonCamera(VisionConstants.LEFT_CAMERA_NAME, VisionConstants.LEFT_CAMERA_POSE);
    rightCam = new BreakerPhotonCamera(VisionConstants.RIGHT_CAMERA_NAME, VisionConstants.RIGHT_CAMERA_POSE);
    backCam = new BreakerPhotonCamera(VisionConstants.BACK_CAMERA_NAME, VisionConstants.BACK_CAMERA_POSE);
    vision = new BreakerVision(VisionConstants.POSE_FILTER_TRUST_COEF, VisionConstants.POSE_FILTER_MAX_UNCERTANTY, VisionConstants.POSE_FILTER_DISTANCE_SCALE_FACTOR, VisionConstants.POSE_FILTER_MAX_DISTANCE, new BreakerPhotonCamera[]{frontCam, leftCam, rightCam, backCam}, VisionConstants.APRILTAG_IDS_AND_LOCATIONS);
  }

  private static void configApriltags() {
    VisionConstants.APRILTAG_IDS_AND_LOCATIONS.put(0, new Pose3d());
  }

  @Override
  public void setOdometryPosition(Pose2d newPose) {
    vision.setOdometryPosition(newPose);
  }

  @Override
  public Pose2d getOdometryPoseMeters() {
    return vision.getOdometryPoseMeters();
  }

  @Override
  public BreakerMovementState2d getMovementState() {
    return vision.getMovementState();
  }

  @Override
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return vision.getRobotRelativeChassisSpeeds();
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return vision.getFieldRelativeChassisSpeeds();
  }

  @Override
  public double getDataTimestamp() {
    return vision.getBaseVisionOdometer().getDataTimestamp();
  }

  @Override
  public boolean isAnyTargetVisable() {
    return vision.getBaseVisionOdometer().isAnyTargetVisable();
  }


}
