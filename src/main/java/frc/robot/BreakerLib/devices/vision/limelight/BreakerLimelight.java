// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.limelight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.BreakerLib.devices.vision.BreakerGenericFiducialTarget;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers.LimelightResults;

/** Add your docs here. */
public class BreakerLimelight implements BreakerGenericVisionOdometer {
    private final String limelightName;
    public BreakerLimelight(String limelightName) {
        this.limelightName = limelightName;
        
    }

    public Pair<Pose3d, Double> getFiducialRobotPoseAndLatancy() {
        double[] data = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
        return new Pair<Pose3d,Double>(new Pose3d(new Translation3d(data[0], data[1], data[2]), new Rotation3d(data[3], data[4], data[5])), data[6] / 1000.0);
    }

    public Pair<Pose3d, Double> getAllianceFiducialRobotPoseAndLatancy() {
        double[] data = NetworkTableInstance.getDefault().getTable("limelight").getEntry(DriverStation.getAlliance() == Alliance.Red ? "botpose_wpired" : "botpose_wpiblue").getDoubleArray(new double[6]);
        return new Pair<Pose3d,Double>(new Pose3d(new Translation3d(data[0], data[1], data[2]), new Rotation3d(data[3], data[4], data[5])), data[6] / 1000.0);
    }



    public LimelightResults getLatestResults() {
        return LimelightHelpers.getLatestResults(limelightName);
    }

    public class BreakerLimelightMegaTagTarget implements BreakerGenericFiducialTarget {
        public BreakerLimelightMegaTagTarget() {

        }

        @Override
        public double getLastTargetFoundTimestamp() {
            return 0;
        }

        @Override
        public double getTargetDataAge() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public Pose3d getCameraPose3d() {
            // TODO Auto-generated method stub
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
        
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getDataTimestamp() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean isAnyTargetVisable() {
        // TODO Auto-generated method stub
        return false;
    }


}
