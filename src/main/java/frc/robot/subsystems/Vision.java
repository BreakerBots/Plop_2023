// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonVision;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.position.odometry.vision.posefilter.BreakerVisionPoseFilterOdometer;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase implements BreakerGenericVisionOdometer {
  /** Creates a new Vision. */
  private BreakerPhotonCamera frontCam, leftCam, rightCam, backCam;
  private BreakerPhotonVision vision;
  private Pose2d curPose;
  public Vision() {
   // frontCam = new BreakerPhotonCamera(VisionConstants.FRONT_CAMERA_NAME, VisionConstants.FRONT_CAMERA_POSE);
    leftCam = new BreakerPhotonCamera(VisionConstants.LEFT_CAMERA_NAME, VisionConstants.LEFT_CAMERA_POSE);
    //rightCam = new BreakerPhotonCamera(VisionConstants.RIGHT_CAMERA_NAME, VisionConstants.RIGHT_CAMERA_POSE);
    //backCam = new BreakerPhotonCamera(VisionConstants.BACK_CAMERA_NAME, VisionConstants.BACK_CAMERA_POSE);
    vision = new BreakerPhotonVision(VisionConstants.POSE_FILTER_TRUST_COEF, VisionConstants.POSE_FILTER_MAX_UNCERTANTY, VisionConstants.POSE_FILTER_DISTANCE_SCALE_FACTOR, VisionConstants.POSE_FILTER_MAX_DISTANCE, new BreakerPhotonCamera[]{leftCam}, VisionConstants.APRILTAG_FIELD_LAYOUT);
    BreakerLog.getInstance().registerLogable("Vision", this);
  curPose = null;
  }

  @Override
  public void setOdometryPosition(Pose2d newPose) {
    vision.setOdometryPosition(newPose);
  }

  @Override
  public Pose2d getOdometryPoseMeters() {
    return curPose;
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
    return vision.getBaseVisionOdometer().isAnyTargetVisable() && Objects.nonNull(curPose); //&& !DriverStation.isAutonomous();
  }

  @Override
  public void periodic() {
    Pose2d pos = vision.getFilteredRobotPose();
    if (vision.getBaseVisionOdometer().isAnyTargetVisable() && Objects.nonNull(pos) && !Double.isNaN(pos.getX()) && !Double.isNaN(pos.getY()) && !Double.isNaN(pos.getRotation().getRadians())) {
      curPose = pos;
    } else {
      curPose = null;
    }
  }

  @Override
  public void toLog(LogTable table) {
    table.put("nullpos", curPose==null);
    if (curPose!=null)
    BreakerGenericVisionOdometer.super.toLog(table);
  }


}
