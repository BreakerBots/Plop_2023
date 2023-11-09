// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.AllianceManager;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.dashboard.BreakerDashboard;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveBase;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModuleBuilder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveCANcoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerTalonFXSwerveModuleDriveMotor.TalonFXControlOutputUnits;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerMath.MirrorSymetryAxis2d;
import frc.robot.Constants.FieldConstants;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.MiscConstants.CANIVORE_1;
import static frc.robot.Constants.PoseEstimationConstants.*;

import java.util.Objects;
import java.util.Optional;

/** Add your docs here. */
public class Drive extends BreakerSwerveDriveBase {

    private BreakerPigeon2 pigeon;
    private Vision vision;

    private static TalonFX driveFL = new TalonFX(FL_DRIVE_ID, CANIVORE_1);
    private static TalonFX turnFL = new TalonFX(FL_TURN_ID, CANIVORE_1);
    private static BreakerSwerveAzimuthEncoder encoderFL = new BreakerSwerveCANcoder(new CANcoder(FL_ENCODER_ID, CANIVORE_1));

    private static TalonFX driveFR = new TalonFX(FR_DRIVE_ID, CANIVORE_1);
    private static TalonFX turnFR = new TalonFX(FR_TURN_ID, CANIVORE_1);
    private static BreakerSwerveAzimuthEncoder encoderFR =  new BreakerSwerveCANcoder(new CANcoder(FR_ENCODER_ID, CANIVORE_1));

    private static TalonFX driveBL = new TalonFX(BL_DRIVE_ID, CANIVORE_1);
    private static TalonFX turnBL = new TalonFX(BL_TURN_ID, CANIVORE_1);
    private static BreakerSwerveAzimuthEncoder encoderBL =  new BreakerSwerveCANcoder(new CANcoder(BL_ENCODER_ID, CANIVORE_1));

    private static TalonFX driveBR = new TalonFX(BR_DRIVE_ID, CANIVORE_1);
    private static TalonFX turnBR = new TalonFX(BR_TURN_ID, CANIVORE_1);
    private static BreakerSwerveAzimuthEncoder encoderBR =  new BreakerSwerveCANcoder(new CANcoder(BR_ENCODER_ID, CANIVORE_1));

    private static BreakerSwerveModule frontLeftModule = BreakerSwerveModuleBuilder.getInstance(MODULE_CONFIG)
        .withTalonFXAngleMotor(turnFL, encoderFL, FL_ENCODER_OFFSET, true)
        .withTalonFXDriveMotor(driveFL, TalonFXControlOutputUnits.VOLTAGE, true)
        .createSwerveModule(FL_TRANSLATION);

    private static BreakerSwerveModule frontRightModule = BreakerSwerveModuleBuilder.getInstance(MODULE_CONFIG)
        .withTalonFXAngleMotor(turnFR, encoderFR, FR_ENCODER_OFFSET, true)
        .withTalonFXDriveMotor(driveFR, TalonFXControlOutputUnits.VOLTAGE, false)
        .createSwerveModule(FR_TRANSLATION);

    private static BreakerSwerveModule backLeftModule = BreakerSwerveModuleBuilder.getInstance(MODULE_CONFIG)
        .withTalonFXAngleMotor(turnBL, encoderBL, BL_ENCODER_OFFSET, true)
        .withTalonFXDriveMotor(driveBL, TalonFXControlOutputUnits.VOLTAGE, true)
        .createSwerveModule(BL_TRANSLATION);

    private static BreakerSwerveModule backRightModule = BreakerSwerveModuleBuilder.getInstance(MODULE_CONFIG)
        .withTalonFXAngleMotor(turnBR, encoderBR, BR_ENCODER_OFFSET, true)
        .withTalonFXDriveMotor(driveBR, TalonFXControlOutputUnits.VOLTAGE, false)
        .createSwerveModule(BR_TRANSLATION);

    private static Field2d field = new Field2d();
    

    public Drive(BreakerPigeon2 pigeon, Vision vision) {
        super(DRIVE_BASE_CONFIG, new BreakerSwerveOdometryConfig(vision, ENCODER_ODOMETRY_STANDARD_DEVATIONS, VISION_ODOMETRY_STANDARD_DEVATIONS), pigeon, frontLeftModule, frontRightModule, backLeftModule, backRightModule);
        this.vision = vision;
        BreakerDashboard.getMainTab().add(field);
        
        frontLeftModule.setDeviceName(" FL_Module ");
        frontRightModule.setDeviceName(" FR_Module ");
        backLeftModule.setDeviceName(" BL_Module ");
        backRightModule.setDeviceName(" BR_Module ");

        BreakerDashboard.getDiagnosticsTab().add("FL Module", frontLeftModule);
        BreakerDashboard.getDiagnosticsTab().add("FR Module", frontRightModule);
        BreakerDashboard.getDiagnosticsTab().add("BL Module", backLeftModule);
        BreakerDashboard.getDiagnosticsTab().add("BR Module", backRightModule);

        BreakerLog.getInstance().registerLogable("Drive", this);

        this.pigeon = pigeon;
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        Optional<Alliance> ally = AllianceManager.getAlliance();
        if (ally.get() == Alliance.Red) {
            newPose = BreakerMath.mirrorPose(newPose, FIELD_LENGTH_X/2.0, MirrorSymetryAxis2d.Y, MirrorSymetryAxis2d.Y);
        } 
        super.setOdometryPosition(newPose);
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        Optional<Alliance> ally = AllianceManager.getAlliance();
        Pose2d pose = getAbsoluteOdometryPoseMeters();
        if (ally.get() == Alliance.Red) {
            pose =  BreakerMath.mirrorPose(pose, FIELD_LENGTH_X/2.0, MirrorSymetryAxis2d.Y, MirrorSymetryAxis2d.Y);
        }
        return pose;
    }

    public Pose2d getAbsoluteOdometryPoseMeters() {
        Pose2d pos = super.getOdometryPoseMeters();
        if(Objects.nonNull(vision) && !Double.isNaN(pos.getX()) && !Double.isNaN(pos.getY()) && !Double.isNaN(pos.getRotation().getRadians()) && vision.isAnyTargetVisable()) {
            pos = vision.getOdometryPoseMeters();
            setAbsoluteOdometryPosition(pos);
            
        }
        return pos;
    }

    public void setAbsoluteOdometryPosition(Pose2d newPose) {
        super.setOdometryPosition(newPose);
    }

    @Override
    public void periodic() {
        Pose2d pos = getOdometryPoseMeters();
        if(!Double.isNaN(pos.getX()) && !Double.isNaN(pos.getY()) && !Double.isNaN(pos.getRotation().getRadians()) && vision.isAnyTargetVisable()) {
            setAbsoluteOdometryPosition(vision.getOdometryPoseMeters());
        }
        field.setRobotPose(pos);
    }

    @Override
    public void toLog(LogTable table) {
        // TODO Auto-generated method stub
        super.toLog(table);
        table.put("IMUPitchDeg", pigeon.getPitch());
        table.put("IMURollDeg", pigeon.getRoll());
    }
}
