// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.limelight;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.devices.vision.BreakerGenericFiducialTarget;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers.LimelightResults;

/** Add your docs here. */
public class BreakerLimelight extends BreakerGenericDevice {
    private final String limelightName;
    private final Transform3d cameraPositionRelativeToRobot;
    public BreakerLimelight(String limelightName, Transform3d cameraPositionRelativeToRobot) {
        this.limelightName = limelightName;
        this.cameraPositionRelativeToRobot = cameraPositionRelativeToRobot;
    }

    public boolean hasTargets() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
    }

    public Pair<Pose3d, Double> getFiducialRobotPoseAndLatancy() {
        double[] data = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[7]);
        return new Pair<Pose3d,Double>(new Pose3d(new Translation3d(data[0], data[1], data[2]), new Rotation3d(data[3], data[4], data[5])), data[6] / 1000.0);
    }

    public Pair<Pose3d, Double> getAllianceFiducialRobotPoseAndLatancy() {
        double[] data = NetworkTableInstance.getDefault().getTable("limelight").getEntry(DriverStation.getAlliance() == Alliance.Red ? "botpose_wpired" : "botpose_wpiblue").getDoubleArray(new double[7]);
        return new Pair<Pose3d,Double>(new Pose3d(new Translation3d(data[0], data[1], data[2]), new Rotation3d(data[3], data[4], data[5])), data[6] / 1000.0);
    }

    public LimelightResults getLatestResults() {
        return LimelightHelpers.getLatestResults(limelightName);
    }

    public static class BreakerLimelightMegaTagTarget extends SubsystemBase implements BreakerGenericFiducialTarget {
        private BreakerLimelight limelight;
        private Transform3d camTrans;
        private ReportedPoseType reportedPoseType;
        private double lastDataUpdate;
        private double latency;
        private Pose3d pose3d;
        private boolean assignedTargetFound = false;
        private boolean assignedTargetFoundInCycle = false; 
        public BreakerLimelightMegaTagTarget(BreakerLimelight limelight, boolean applyCodeCameraTransform, ReportedPoseType reportedPoseType) {
            assignedTargetFound = false;
            assignedTargetFoundInCycle = false; 
        }

        @Override
        public void periodic() {
            if (limelight.hasTargets()) {
                assignedTargetFound = true;
                assignedTargetFoundInCycle = true;
                lastDataUpdate = Timer.getFPGATimestamp();
                Pair<Pose3d, Double> pair;
                if (reportedPoseType == ReportedPoseType.ALLIANCE_ORIGIN_BOTPOSE) {
                    pair = limelight.getAllianceFiducialRobotPoseAndLatancy();
                } else {
                    pair = limelight.getFiducialRobotPoseAndLatancy();
                }
                pose3d = pair.getFirst().plus(camTrans);
                latency = pair.getSecond();
            } else {
                assignedTargetFoundInCycle = false;
            }
        }

        @Override
        public double getLastTargetFoundTimestamp() {
            return lastDataUpdate;
        }

        @Override
        public double getTargetDataAge() {
            double timediffsec = Timer.getFPGATimestamp() - lastDataUpdate;
            return Units.millisecondsToSeconds(latency) + timediffsec;
        }

        @Override
        public Pose3d getCameraPose3d() {
            return null;
        }

        @Override
        public Pose3d getRobotPose3d() {
            return null;
        }

        @Override
        public double getYaw() {
            return 0;
        }

        @Override
        public double getPitch() {
            return 0;
        }

        @Override
        public double getRobotRelativeYaw() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public double getRobotRelativePitch() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public double getSkew() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public double getArea() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public boolean getAssignedTargetFound() {
            // TODO Auto-generated method stub
            return false;
        }

        @Override
        public boolean isAssignedTargetVisible() {
            // TODO Auto-generated method stub
            return false;
        }

        @Override
        public double getDistance2D() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public double getDistance() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public int getFiducialID() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public double getPoseAmbiguity() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public AprilTag getBaseApriltag() {
            // TODO Auto-generated method stub
            return null;
        }
        
        public static enum ReportedPoseType {
            FIELD_ORIGIN_BOTPOSE,
            ALLIANCE_ORIGIN_BOTPOSE
        }
    }

    @Override
    public void runSelfTest() {
        // TODO Auto-generated method stub
        
    }

  


}
