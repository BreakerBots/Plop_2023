// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.motorgroups;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.differential.BreakerDiffDriveFusedVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.differential.BreakerDiffDriveOdometer;
import frc.robot.BreakerLib.position.odometry.differential.BreakerDiffDriveState;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDriveConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveOdometryConfig;

public class BreakerDiffDrive extends BreakerGenericDrivetrain {
  /** Creates a new BreakerDiffDrive. */
  protected BreakerDiffDriveMotorGroup leftMotorGroup, rightMotorGroup;
  private BreakerDiffDriveConfig config;
  public BreakerDiffDrive(BreakerDiffDriveConfig config, BreakerSwerveOdometryConfig odometryConfig, BreakerGenericGyro gyro, BreakerDiffDriveMotorGroup leftMotorGroup, BreakerDiffDriveMotorGroup rightMotorGroup) {

  }

  public BreakerDiffDrive(BreakerDiffDriveConfig config, BreakerGenericGyro gyro, BreakerDiffDriveMotorGroup leftMotorGroup, BreakerDiffDriveMotorGroup rightMotorGroup) {

  }



  public DifferentialDriveKinematics getKinematics() {
    return config.getKinematics();
  }

  public double getLeftDriveWheelSpeed() {
    return leftMotorGroup.getRotorVelocity() / config.getEncoderRotationsPerMeter();
  }

  public double getLeftDriveWheelDistance() {
    return leftMotorGroup.getRotorPosition() / config.getEncoderRotationsPerMeter();
  }

  public double getRightDriveWheelSpeed() {
    return leftMotorGroup.getRotorVelocity() / config.getEncoderRotationsPerMeter();
  }

  public double getRightDriveWheelDistance() {
    return rightMotorGroup.getRotorPosition() / config.getEncoderRotationsPerMeter();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftDriveWheelSpeed(), getRightDriveWheelSpeed());
  }

  public DifferentialDriveWheelVoltages getWheelVoltages() {
    return new DifferentialDriveWheelVoltages(leftMotorGroup.getOutputVoltage() * RobotController.getBatteryVoltage(), rightMotorGroup.getOutputVoltage() * RobotController.getBatteryVoltage());
  }

  public BreakerDiffDriveState getDriveState() {
    return new BreakerDiffDriveState(getWheelSpeeds(), getLeftDriveWheelDistance(), getLeftDriveWheelDistance());
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
  public void runSelfTest() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public BreakerGenericGyro getBaseGyro() {
  
    return null;
  }

  @Override
  public void setDrivetrainBrakeMode(boolean isEnabled) {
    leftMotorGroup.setBrakeMode(isEnabled);
    rightMotorGroup.setBrakeMode(isEnabled);
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds(BreakerGenericOdometer odometer) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static class BreakerDiffDriveOdometryConfig {
    private BreakerGenericVisionOdometer vision;
    private Pose2d initalPose;
    private double[] stateStanderdDeveation, visionStanderdDeveation;
    private boolean usePoseEstimator;

    public BreakerDiffDriveOdometryConfig() {
       this(new Pose2d());
    }

    public BreakerDiffDriveOdometryConfig(Pose2d initalPose) {
        this.initalPose = initalPose;
        usePoseEstimator = false;
    }

    public BreakerDiffDriveOdometryConfig(
        BreakerGenericVisionOdometer vision,
        Pose2d initalPose,
        double[] stateStanderdDeveation,
        double[] visionStanderdDeveation
        ) {
       this.initalPose = initalPose;
       this.vision = vision;
       this.stateStanderdDeveation = stateStanderdDeveation;
       this.visionStanderdDeveation = visionStanderdDeveation;
       usePoseEstimator = true;
    }

    public BreakerDiffDriveOdometryConfig(
        BreakerGenericVisionOdometer vision,
        double[] stateStanderdDeveation,
        double[] visionStanderdDeveation
        ) {
        this(vision, new Pose2d(), stateStanderdDeveation, visionStanderdDeveation);
    }

    public BreakerGenericOdometer getOdometer(BreakerDiffDrive drivetrain) {
        if (usePoseEstimator) {
            return new BreakerDiffDriveFusedVisionPoseEstimator(drivetrain, vision, initalPose, visionStanderdDeveation, stateStanderdDeveation);
        }
        return new BreakerDiffDriveOdometer(drivetrain, initalPose);
    }
  }


}
