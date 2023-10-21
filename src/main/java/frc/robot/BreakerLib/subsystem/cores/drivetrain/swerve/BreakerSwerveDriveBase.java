// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.auto.pathplanner.BreakerSwervePathFollower;
import frc.robot.BreakerLib.auto.pathplanner.BreakerSwervePathFollower.BreakerSwervePathFollowerConfig;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwervePercentSpeedRequest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwerveVelocityRequest;

/** Add your docs here. */
public class  BreakerSwerveDriveBase extends BreakerSwerveDrive {
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
        pathFollowerConfig = new BreakerSwervePathFollowerConfig(this, false);
    }

    @Override
    protected void move(ChassisSpeeds targetChassisSpeeds, SwerveMovementRefrenceFrame swerveMovementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation, boolean headingCorrectionEnabled, boolean isOpenLoop) {
        ChassisSpeeds targetVels = new ChassisSpeeds(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond, targetChassisSpeeds.omegaRadiansPerSecond);
        Rotation2d curAng = getOdometryPoseMeters().getRotation();
        switch(swerveMovementRefrenceFrame) {
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
            if (headingCorrectionEnabled) {
                targetVels.omegaRadiansPerSecond = config.getHeadingCompensationController().calculate(curAng.getRadians(), lastSetHeading.getRadians());
            }
        } else {
            lastSetHeading = curAng;
        }

        if (slowModeValue == SlowModeValue.ENABLED || (slowModeValue == SlowModeValue.DEFAULT && slowModeActive)) {
            targetVels.vxMetersPerSecond *= config.getSlowModeLinearMultiplier();
            targetVels.vyMetersPerSecond *= config.getSlowModeLinearMultiplier();
            targetVels.omegaRadiansPerSecond *= config.getSlowModeTurnMultiplier();
        }

        setModuleStates(false, isOpenLoop, getKinematics().toSwerveModuleStates(targetVels, centerOfRotation));
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

    public static class BreakerSwerveDriveBaseVelocityRequest extends BreakerSwerveVelocityRequest {

        public BreakerSwerveDriveBaseVelocityRequest(ChassisSpeeds speeds) {
            super(speeds, SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET, SlowModeValue.DEFAULT,  new Translation2d(), false, true);
        }

        public BreakerSwerveDriveBaseVelocityRequest(ChassisSpeeds speeds,
                SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue,
                Translation2d centerOfRotation, boolean isOpenLoop, boolean headingCorrectionEnabled) {
            super(speeds, movementRefrenceFrame, slowModeValue, centerOfRotation, isOpenLoop, headingCorrectionEnabled);
        }

    }

    public static class BreakerSwerveDriveBasePercentSpeedRequest extends BreakerSwervePercentSpeedRequest {

        public BreakerSwerveDriveBasePercentSpeedRequest(ChassisPercentSpeeds percentSpeeds) {
            super(percentSpeeds, SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET, SlowModeValue.DEFAULT,  new Translation2d(), false, true);
        }

        public BreakerSwerveDriveBasePercentSpeedRequest(ChassisPercentSpeeds percentSpeeds,
                SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue,
                Translation2d centerOfRotation, boolean isOpenLoop, boolean headingCorrectionEnabled) {
            super(percentSpeeds, movementRefrenceFrame, slowModeValue, centerOfRotation, isOpenLoop, headingCorrectionEnabled);
        }

    }

    public static class BreakerSwerveDriveBaseConfig extends BreakerSwerveDriveConfig {
        private PIDController xController, yController, thetaController, headingCompensationController;
        private PPHolonomicDriveController driveController;
        private double headingCompensationAngularVelDeadband, headingCompensationMinActiveLinearSpeed;
        public BreakerSwerveDriveBaseConfig(double maxLinearVel, double maxAngVel, 
                double headingCompensationAngularVelDeadband, double headingCompensationMinActiveLinearSpeed, double moduleWheelSpeedDeadband,
                PIDController xController, PIDController yController, PIDController thetaController) {
            super(maxLinearVel, maxAngVel, moduleWheelSpeedDeadband);
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
