// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveDrivePoseEstimator;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers.LimelightResults;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers.LimelightTarget_Fiducial;

/** Add your docs here. */
public class Vision2 extends BreakerSwerveDrivePoseEstimator {
    private PhotonCamera frontCam, leftCam, backCam;
    private PhotonPoseEstimator frontEst, leftEst, backEst;
    private BreakerSwerveDrivePoseEstimator poseEst;
    private double frontLastTs, leftLastTs, backLastTs, llLastTs;
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(8, 8, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    private boolean odomSeeded = false;
    
    public Vision2(BreakerSwerveDrive drive) {
        super(drive, new Pose2d(), Constants.PoseEstimationConstants.ENCODER_ODOMETRY_STANDARD_DEVATIONS, Constants.PoseEstimationConstants.VISION_ODOMETRY_STANDARD_DEVATIONS);
        frontCam = new PhotonCamera(Constants.VisionConstants.FRONT_CAMERA_NAME);
        leftCam = new PhotonCamera(Constants.VisionConstants.LEFT_CAMERA_NAME);
        backCam = new PhotonCamera(Constants.VisionConstants.BACK_CAMERA_NAME);
        frontEst = new PhotonPoseEstimator(Constants.VisionConstants.APRILTAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP, frontCam, Constants.VisionConstants.FRONT_CAMERA_POSE);
        leftEst = new PhotonPoseEstimator(Constants.VisionConstants.APRILTAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP, leftCam, Constants.VisionConstants.LEFT_CAMERA_POSE);
        backEst = new PhotonPoseEstimator(Constants.VisionConstants.APRILTAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP, backCam, Constants.VisionConstants.BACK_CAMERA_POSE);
    }

    public Optional<EstimatedRobotPose> getFrontEstimatedGlobalPose() {
        var visionEst = frontEst.update();
        double latestTimestamp = frontCam.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - frontLastTs) > 1e-5;
        if (newResult) frontLastTs = latestTimestamp;
        return visionEst;
    }

    public Optional<EstimatedRobotPose> getLeftEstimatedGlobalPose() {
        var visionEst = leftEst.update();
        double latestTimestamp = frontCam.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - leftLastTs) > 1e-5;
        if (newResult) leftLastTs = latestTimestamp;
        return visionEst;
    }

    public Optional<EstimatedRobotPose> getBackEstimatedGlobalPose() {
        var visionEst = backEst.update();
        double latestTimestamp = frontCam.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - backLastTs) > 1e-5;
        if (newResult) frontLastTs = latestTimestamp;
        return visionEst;
    }

    public Optional<LimelightResults> getLimelightEstPose() {
        LimelightResults results = LimelightHelpers.getLatestResults("limelight");
        double latestTs = results.targetingResults.timestamp_LIMELIGHT_publish;
        boolean newResult = Math.abs(latestTs - llLastTs) > 1e-5;
        Optional<LimelightResults> estPose = Optional.empty();
        if (newResult && results.targetingResults.valid && results.targetingResults.targets_Fiducials.length > 0) {
            llLastTs = latestTs;
            estPose = Optional.of(results);
        }
        return estPose;
    }

    
    
    public Matrix<N3, N1> getEstimationStdDevsPV(EstimatedRobotPose estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = estimatedPose.targetsUsed;
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = Constants.VisionConstants.APRILTAG_FIELD_LAYOUT.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.estimatedPose.getTranslation().toTranslation2d());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public Matrix<N3, N1> getEstimationStdDevsLL(Pose2d estBotPose, LimelightTarget_Fiducial... fiducialTargets) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = fiducialTargets;
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = Constants.VisionConstants.APRILTAG_FIELD_LAYOUT.getTagPose((int) tgt.fiducialID);
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estBotPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    private void addVisionMeasurementLL(Optional<LimelightResults> estPoseOpt) {
        if (estPoseOpt.isPresent()) {
            if (odomSeeded) {
                LimelightResults estPose = estPoseOpt.get();
                addVisionMeasurment(estPose.targetingResults.getBotPose2d_wpiBlue(), estPose.targetingResults.timestamp_LIMELIGHT_publish, getEstimationStdDevsLL(estPose.targetingResults.getBotPose2d_wpiBlue(), estPose.targetingResults.targets_Fiducials)); 
            } else {
                setOdometryPosition(estPoseOpt.get().targetingResults.getBotPose2d_wpiBlue());
            }
        }
    }

    private void addVisionMeasurement( Optional<EstimatedRobotPose> estPoseOpt) {
        if (estPoseOpt.isPresent()) {
            if (odomSeeded) {
                EstimatedRobotPose estPose = estPoseOpt.get();
                addVisionMeasurment(estPose.estimatedPose.toPose2d(), estPose.timestampSeconds, getEstimationStdDevsPV(estPose)); 
            } else {
                setOdometryPosition(estPoseOpt.get().estimatedPose.toPose2d());
            }
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        addVisionMeasurement(getFrontEstimatedGlobalPose());
        addVisionMeasurement(getLeftEstimatedGlobalPose());
        addVisionMeasurement(getBackEstimatedGlobalPose());
        addVisionMeasurementLL(getLimelightEstPose());
        

    }

   }
