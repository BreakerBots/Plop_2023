// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.motorgroups.BreakerDiffDriveMotorGroup;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveOdometryConfig;

/** Add your docs here. */
public class BreakerDiffDriveBase extends BreakerDiffDrive {
    private Rotation2d lastSetHeading;
    private BreakerDiffDriveBaseConfig config;

    public BreakerDiffDriveBase(BreakerDiffDriveBaseConfig config, BreakerDiffDriveOdometryConfig odometryConfig,
            BreakerGenericGyro gyro, BreakerDiffDriveMotorGroup leftMotorGroup,
            BreakerDiffDriveMotorGroup rightMotorGroup) {
        super(config, odometryConfig, gyro, leftMotorGroup, rightMotorGroup);
        lastSetHeading = getOdometryPoseMeters().getRotation();
        this.config = config;
    }

    public BreakerDiffDriveBase(BreakerDiffDriveBaseConfig config, BreakerGenericGyro gyro,
            BreakerDiffDriveMotorGroup leftMotorGroup, BreakerDiffDriveMotorGroup rightMotorGroup) {
        this(config, new BreakerDiffDriveOdometryConfig(), gyro, leftMotorGroup, rightMotorGroup);
        this.config = config;
    }

    @Override
    public void arcadeDrive(double xSpeed, double zRotation, BreakerDiffDriveMovementPrefrences movementPrefrences) {
        WheelSpeeds wheelSpeeds = arcadeDriveIK(xSpeed, zRotation, movementPrefrences);
        wheelSpeeds = applyHeadingCorrection(wheelSpeeds, movementPrefrences.getHeadingCorrectionEnabled());
        setRawWheelSpeeds(wheelSpeeds);
    }

    @Override
    public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace,
            BreakerDiffDriveMovementPrefrences movementPrefrences) {
                WheelSpeeds wheelSpeeds = curvatureDriveIK(xSpeed, zRotation, allowTurnInPlace, movementPrefrences);
                wheelSpeeds = applyHeadingCorrection(wheelSpeeds, movementPrefrences.getHeadingCorrectionEnabled());
                setRawWheelSpeeds(wheelSpeeds);
    }

    @Override
    public void tankDrive(double leftSpeed, double rightSpeed, BreakerDiffDriveMovementPrefrences movementPrefrences) {
        WheelSpeeds wheelSpeeds = arcadeDriveIK(leftSpeed, rightSpeed, movementPrefrences);
        wheelSpeeds = applyHeadingCorrection(wheelSpeeds, movementPrefrences.getHeadingCorrectionEnabled());
        setRawWheelSpeeds(wheelSpeeds);
    }

    private WheelSpeeds applyHeadingCorrection(WheelSpeeds wheelSpeeds, boolean applyHeadingCompensation) {
        Pair<Double, Double> arcadeSpeeds = wheelSpeedsToArcadeSpeeds(wheelSpeeds);
        double xSpeed = arcadeSpeeds.getFirst();
        double zRotation = arcadeSpeeds.getSecond();
        Rotation2d curAng = getOdometryPoseMeters().getRotation();
        if (Math.abs(zRotation) < config.getHeadingCompensationAngularVelDeadband() && Math.abs(xSpeed) > config.getHeadingCompensationMinActiveLinearSpeed()) {
            if (applyHeadingCompensation) {
                zRotation = config.getHeadingCompensationController().calculate(curAng.getRadians(), lastSetHeading.getRadians());
            }
        } else {
            lastSetHeading = curAng;
        }
        return DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, false);
    }

    @Override
    public BreakerDiffDriveBaseConfig getConfig() {
        return config;
    }

    /** xSpeed, zRotation */
    private static Pair<Double, Double> wheelSpeedsToArcadeSpeeds(WheelSpeeds wheelSpeeds) {
        return new Pair<Double,Double>(null, null);
    }


    protected static class BreakerDiffDriveBaseMovementPrefrences extends BreakerDiffDriveMovementPrefrences {

        protected BreakerDiffDriveBaseMovementPrefrences(boolean headingCorrectionEnabled, boolean squareInputsEnabled,
                double inputDeadband, SlowModeValue slowModeValue) {
            super(headingCorrectionEnabled, squareInputsEnabled, inputDeadband, slowModeValue);
        }

        public BreakerDiffDriveBaseMovementPrefrences withHeadingCorrectionEnabled(boolean isEnabled) {
            return new BreakerDiffDriveBaseMovementPrefrences(isEnabled, squareInputsEnabled, inputDeadband, slowModeValue);
        }

        @Override
        public BreakerDiffDriveBaseMovementPrefrences withInputDeadband(double deadband) {
            return new BreakerDiffDriveBaseMovementPrefrences(headingCorrectionEnabled, squareInputsEnabled, deadband, slowModeValue);
        }

        @Override
        public BreakerDiffDriveBaseMovementPrefrences withSlowModeValue(SlowModeValue slowModeValue) {
            return new BreakerDiffDriveBaseMovementPrefrences(headingCorrectionEnabled, squareInputsEnabled, inputDeadband, slowModeValue);
        }
        
        @Override
        public BreakerDiffDriveBaseMovementPrefrences withSquareInputsEnabled(boolean isEnabled) {
            return new BreakerDiffDriveBaseMovementPrefrences(headingCorrectionEnabled, isEnabled, inputDeadband, slowModeValue);
        }
    }

    public static class BreakerDiffDriveBaseConfig extends BreakerDiffDriveConfig {
        private PIDController leftController, rightController, headingCompensationController;
        private double headingCompensationAngularVelDeadband, headingCompensationMinActiveLinearSpeed;
        public BreakerDiffDriveBaseConfig(
            double gearRatioTo1, double wheelDiameterMeters, double robotTrackWidthMeters, 
            double headingCompensationAngularVelDeadband, double headingCompensationMinActiveLinearSpeed,
            PIDController leftController, PIDController rightController, PIDController headingCompensationController
            ) {
            super(gearRatioTo1, wheelDiameterMeters, robotTrackWidthMeters);
            
        }

        public PIDController getLeftController() {
            return leftController;
        }

        public PIDController getRightController() {
            return rightController;
        }

        public PIDController getHeadingCompensationController() {
            return headingCompensationController;
        }

        public double getHeadingCompensationAngularVelDeadband() {
            return headingCompensationAngularVelDeadband;
        }

        public double getHeadingCompensationMinActiveLinearSpeed() {
            return headingCompensationMinActiveLinearSpeed;
        }
    }

    
    
}
