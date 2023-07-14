// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.HashMap;
import java.util.Objects;

import javax.management.loading.PrivateClassLoader;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.auto.pathplanner.BreakerSwervePathFollower;
import frc.robot.BreakerLib.auto.pathplanner.BreakerSwervePathFollower.BreakerSwervePathFollowerConfig;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveDriveFusedVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveBase.BreakerSwerveDriveBaseMovementPreferences;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;

/** Add your docs here. */
public class BreakerSwerveDriveBase extends BreakerSwerveDrive {
    private BreakerSwerveDriveBaseConfig config;
    private BreakerSwervePathFollowerConfig pathFollowerConfig;
    private Rotation2d lastSetHeading;
    public BreakerSwerveDriveBase(
            BreakerSwerveDriveBaseConfig config, BreakerGenericGyro gyro,
            BreakerGenericSwerveModule... swerveModules) {
        this(config, new BreakerSwerveOdometryConfig(), gyro, swerveModules);
    }

    public BreakerSwerveDriveBase(
            BreakerSwerveDriveBaseConfig config, 
            BreakerSwerveOdometryConfig odometryConfig, 
            BreakerGenericGyro gyro, 
            BreakerGenericSwerveModule... swerveModules) {
        super(config, odometryConfig, gyro, swerveModules);
        this.config = config;
        lastSetHeading = getOdometryPoseMeters().getRotation();
        pathFollowerConfig = new BreakerSwervePathFollowerConfig(this, config.getDriveController(), false);
    }

    @Override
    public void move(ChassisSpeeds targetChassisSpeeds, BreakerSwerveMovementPreferences movementPreferences) {
        ChassisSpeeds targetVels = new ChassisSpeeds(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond, targetChassisSpeeds.omegaRadiansPerSecond);
        Rotation2d curAng = getOdometryPoseMeters().getRotation();
        switch(movementPreferences.getSwerveMovementRefrenceFrame()) {
            case FIELD_RELATIVE_WITHOUT_OFFSET:
                targetVels = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds, curAng);
                break;
            case FIELD_RELATIVE_WITH_OFFSET:
                targetVels = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds,
                    curAng.plus(getFieldRelativeMovementOffsetAngle()));
                break;
            case ROBOT_RELATIVE:
            default:
                break;
        }

        if (Math.abs(targetChassisSpeeds.omegaRadiansPerSecond) < config.getHeadingCompensationAngularVelDeadband() && Math.hypot(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond) > config.getHeadingCompensationMinActiveLinearSpeed()) {
            if (movementPreferences.getHeadingCorrectionEnabled()) {
                targetVels.omegaRadiansPerSecond = config.getHeadingCompensationController().calculate(curAng.getRadians(), lastSetHeading.getRadians());
            }
        } else {
            lastSetHeading = curAng;
        }

        if (movementPreferences.getSlowModeValue() == SlowModeValue.ENABLED || (movementPreferences.slowModeValue == SlowModeValue.DEFAULT && slowModeActive)) {
            targetVels.vxMetersPerSecond *= config.getSlowModeLinearMultiplier();
            targetVels.vyMetersPerSecond *= config.getSlowModeLinearMultiplier();
            targetVels.omegaRadiansPerSecond *= config.getSlowModeTurnMultiplier();
        }

        setModuleStates(getKinematics().toSwerveModuleStates(targetVels));
    }

    @Override
    public void move(ChassisSpeeds targetChassisSpeeds) {
        move(targetChassisSpeeds, BreakerSwerveDriveBaseMovementPreferences.FIELD_RELATIVE_WITH_OFFSET_AND_HEADING_CORRECTION);
    }

    @Override
    public void move(double percentX, double percentY, double percentOmega,
            BreakerSwerveMovementPreferences movementPreferences) {
        move(percentsToChassisSpeeds(percentX, percentY, percentOmega), BreakerSwerveDriveBaseMovementPreferences.FIELD_RELATIVE_WITH_OFFSET_AND_HEADING_CORRECTION);
    }
    
    @Override
    public void move(double percentX, double percentY, double percentOmega) {
        move(percentX, percentY, percentOmega, BreakerSwerveDriveBaseMovementPreferences.FIELD_RELATIVE_WITH_OFFSET_AND_HEADING_CORRECTION);
    } 

    public BreakerSwervePathFollower followPathCommand(PathPlannerTrajectory path) {
        return new BreakerSwervePathFollower(pathFollowerConfig, path, true);
    }

    public FollowPathWithEvents followPathWithEventsCommand(PathPlannerTrajectory path, HashMap<String, Command> eventMap) {
        return new FollowPathWithEvents(followPathCommand(path), path.getMarkers(), eventMap);
    }

    @Override
    public BreakerSwerveDriveBaseConfig getConfig() {
        return config;
    }

    @Override
    public void setOdometer(BreakerGenericOdometer odometer) {
        super.setOdometer(odometer);
        lastSetHeading = getOdometryPoseMeters().getRotation();
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        super.setOdometryPosition(newPose);
        lastSetHeading = newPose.getRotation();
    }

    @Override
    public void periodic() {
        super.periodic();
        if (DriverStation.isDisabled()) {
            lastSetHeading = getOdometryPoseMeters().getRotation();
        }
    }

    public static class BreakerSwerveDriveBaseMovementPreferences extends BreakerSwerveMovementPreferences {
        public static final BreakerSwerveDriveBaseMovementPreferences FIELD_RELATIVE_WITH_OFFSET_AND_HEADING_CORRECTION = new BreakerSwerveDriveBaseMovementPreferences(SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET, SlowModeValue.DEFAULT, true);
        public static final BreakerSwerveDriveBaseMovementPreferences FIELD_RELATIVE_WITHOUT_OFFSET_AND_WITH_HEADING_CORRECTION = new BreakerSwerveDriveBaseMovementPreferences(SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITHOUT_OFFSET, SlowModeValue.DEFAULT, true);
        public static final BreakerSwerveDriveBaseMovementPreferences ROBOT_RELATIVE_WITH_HEADING_CORRECTION = new BreakerSwerveDriveBaseMovementPreferences(SwerveMovementRefrenceFrame.ROBOT_RELATIVE, SlowModeValue.DEFAULT, true);
    
        public BreakerSwerveDriveBaseMovementPreferences(SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, boolean headingCorrectionEnabled) {
          super(movementRefrenceFrame, slowModeValue, headingCorrectionEnabled);
        }
    

        public BreakerSwerveDriveBaseMovementPreferences withHeadingCorrectionEnabled(boolean headingCorrectionEnabled) {
            return new BreakerSwerveDriveBaseMovementPreferences(this.swerveMovementRefrenceFrame, this.slowModeValue, headingCorrectionEnabled);
        }

    }

    public static class BreakerSwerveDriveBaseConfig extends BreakerSwerveDriveConfig {
        private PIDController xController, yController, thetaController, headingCompensationController;
        private PPHolonomicDriveController driveController;
        private double headingCompensationAngularVelDeadband, headingCompensationMinActiveLinearSpeed;
        public BreakerSwerveDriveBaseConfig(double maxForwardVel, double maxSidewaysVel, double maxAngVel, 
                double headingCompensationAngularVelDeadband, double headingCompensationMinActiveLinearSpeed, double moduleWheelSpeedDeadband, double maxAttainableModuleWheelSpeed,
                PIDController xController, PIDController yController, PIDController thetaController) {
            super(maxForwardVel, maxSidewaysVel, maxAngVel, moduleWheelSpeedDeadband, maxAttainableModuleWheelSpeed);
            this.xController = xController;
            this.yController = yController;
            this.thetaController = thetaController;
            this.headingCompensationAngularVelDeadband = headingCompensationAngularVelDeadband;
            this.headingCompensationMinActiveLinearSpeed = headingCompensationMinActiveLinearSpeed;
            headingCompensationController = new PIDController(thetaController.getP(), thetaController.getI(), thetaController.getD());
            headingCompensationController.enableContinuousInput(-Math.PI, Math.PI);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
            driveController = new PPHolonomicDriveController(xController, yController, thetaController);
        }

        public PIDController getPIDControllerX() {
            return xController;
        }

        public PIDController getPIDControllerY() {
            return yController;
        }

        public PIDController getPIDControllerTheta() {
            return thetaController;
        }

        public PIDController getHeadingCompensationController() {
            return headingCompensationController;
        }

        public PPHolonomicDriveController getDriveController() {
            return driveController;
        }

        public double getHeadingCompensationAngularVelDeadband() {
            return headingCompensationAngularVelDeadband;
        }

        public double getHeadingCompensationMinActiveLinearSpeed() {
            return headingCompensationMinActiveLinearSpeed;
        }

        @Override
        public BreakerSwerveDriveBaseConfig setSlowModeMultipliers(double linearMulitplier, double turnMultiplier) {
            super.setSlowModeMultipliers(linearMulitplier, turnMultiplier);
            return this;
        }

    }

}
