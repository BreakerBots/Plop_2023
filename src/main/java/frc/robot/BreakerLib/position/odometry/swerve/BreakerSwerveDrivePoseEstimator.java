// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.swerve;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLogUtil;
import frc.robot.BreakerLib.util.math.BreakerMath;
public class BreakerSwerveDrivePoseEstimator extends SubsystemBase implements BreakerGenericOdometer  {
    private BreakerGenericGyro gyro;
    private SwerveDrivePoseEstimator poseEstimator;
    private BreakerSwerveDrive drivetrain;
    private double lastUpdateTimestamp;
    private Pose2d prevPose;
    private Pose2d lastVisionPose;
    private double lastVisionTimestamp;
    private ChassisSpeeds fieldRelativeChassisSpeeds = new ChassisSpeeds();
    private BreakerMovementState2d prevMovementState = new BreakerMovementState2d();
    private BreakerMovementState2d curMovementState = new BreakerMovementState2d();

    public BreakerSwerveDrivePoseEstimator(
        BreakerSwerveDrive drivetrain,
        Pose2d initialPose, 
        double[] stateStandardDeveation, 
        double[] visionStanderdDeveation
    ) {
        this.drivetrain = drivetrain;
        gyro = drivetrain.getBaseGyro();
        poseEstimator = new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
            gyro.getYawRotation2d(), 
            drivetrain.getSwerveModulePositions(),
            initialPose,
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(stateStandardDeveation),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStanderdDeveation));
        prevPose = getOdometryPoseMeters();
        lastVisionPose = getOdometryPoseMeters();
        lastUpdateTimestamp = Timer.getFPGATimestamp();
    }

    public Pose2d addVisionMeasurment(Pose2d robotPoseFromVision, double visionDataTimestamp) {
        poseEstimator.addVisionMeasurement(robotPoseFromVision,
                visionDataTimestamp);
        lastVisionPose = robotPoseFromVision;
        lastVisionTimestamp = visionDataTimestamp;
        return poseEstimator.getEstimatedPosition();
    }

    public void changeVisionDevs(double visionStrdDevX, double visionStrdDevY, double visionStrdDevTheta) {
        poseEstimator.setVisionMeasurementStdDevs(
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStrdDevX, visionStrdDevY, visionStrdDevTheta));
    }

    @Override
    public String toString() {
        return "CURRENT POSE: " + getOdometryPoseMeters().toString();
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        poseEstimator.resetPosition(gyro.getYawRotation2d(), drivetrain.getSwerveModulePositions(), newPose);
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        return curMovementState;
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeChassisSpeeds.vxMetersPerSecond, 
            fieldRelativeChassisSpeeds.vyMetersPerSecond, 
            fieldRelativeChassisSpeeds.omegaRadiansPerSecond, 
            getOdometryPoseMeters().getRotation());
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return fieldRelativeChassisSpeeds;
    }

    private void updateChassisSpeeds() {
        double timeDiff = Timer.getFPGATimestamp() - lastUpdateTimestamp;
        double xSpeed = (getOdometryPoseMeters().getX() - prevPose.getX()) * timeDiff;
        double ySpeed = (getOdometryPoseMeters().getY() - prevPose.getY()) * timeDiff;
        double thetaSpeed = (getOdometryPoseMeters().getRotation().getRadians() - prevPose.getRotation().getRadians()) * timeDiff;
        fieldRelativeChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(getOdometryPoseMeters(), fieldRelativeChassisSpeeds, timeDiff, prevMovementState);
    }

    @Override
    public void periodic() {
        prevPose = getOdometryPoseMeters();
        poseEstimator.update(gyro.getYawRotation2d(), drivetrain.getSwerveModulePositions());
        updateChassisSpeeds();
        lastUpdateTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void toLog(LogTable table) {
        BreakerGenericOdometer.super.toLog(table);
        LogTable visionTable = table.getSubtable("Vision");
        visionTable.put("LastAddedVisionPose", BreakerLogUtil.formatPose2dForLog(lastVisionPose));
        visionTable.put("LastAddedVisionMeasurmentTimestamp", lastVisionTimestamp);
    }

}
