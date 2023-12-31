// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.control.BreakerHolonomicDriveController;
import frc.robot.BreakerLib.control.BreakerProfiledHolonomicDriveControler;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveOdometryConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.SwerveMovementRefrenceFrame;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveBase.BreakerSwerveDriveBaseConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveBase.BreakerSwerveDriveBaseVelocityRequest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModuleBuilder.BreakerSwerveModuleConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwerveVelocityRequest;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
      // Drive motor IDs
      public static final int FL_DRIVE_ID = 10;
      public static final int FR_DRIVE_ID = 12;
      public static final int BL_DRIVE_ID = 14;
      public static final int BR_DRIVE_ID = 16;

      //Azimuth motor IDs
      public static final int FL_TURN_ID = 11;
      public static final int FR_TURN_ID = 13;
      public static final int BL_TURN_ID = 15;
      public static final int BR_TURN_ID = 17;

      //Azimuth Encoder IDs
      public static final int FL_ENCODER_ID = 20;
      public static final int FR_ENCODER_ID = 21;
      public static final int BL_ENCODER_ID = 22;
      public static final int BR_ENCODER_ID = 23;

      //Azimuth encoder angle offets
      public static final double FL_ENCODER_OFFSET = -0.4189453125;
      public static final double FR_ENCODER_OFFSET = -0.238037;
      public static final double BL_ENCODER_OFFSET = 0.11865234375;
      public static final double BR_ENCODER_OFFSET = 0.33837890625;

      //Module wheel centerpoint locations relative to robot origin (center)
      public static final Translation2d FL_TRANSLATION = new Translation2d(0.2635, 0.2635);
      public static final Translation2d FR_TRANSLATION = new Translation2d(0.2635, -0.2635);
      public static final Translation2d BL_TRANSLATION = new Translation2d(-0.2635, 0.2635);
      public static final Translation2d BR_TRANSLATION = new Translation2d(-0.2635, -0.2635);

      //Module Azimuth PIDF constants
      public static final double MODULE_AZIMUTH_KP = 20.419941348973605;
      public static final double MODULE_AZIMUTH_KI = 0.0;
      public static final double MODULE_AZIMUTH_KD = 0.0;
      public static final double MODULE_AZIMUTH_KF = 0.0;
      public static final BreakerSwerveMotorPIDConfig MODULE_ANGLE_PID_CONFIG = new BreakerSwerveMotorPIDConfig(MODULE_AZIMUTH_KP, MODULE_AZIMUTH_KI, MODULE_AZIMUTH_KD, MODULE_AZIMUTH_KF);

      //Module Drive Velocity PIDF constants
      public static final double MODULE_VELOCITY_KP = -0.10810557184750733;
      public static final double MODULE_VELOCITY_KI = 0.0;
      public static final double MODULE_VELOCITY_KD = 0.0;
      public static final double MODULE_VELOCITY_KF = 0.0;
      public static final BreakerSwerveMotorPIDConfig MODULE_VELOCITY_PID_CONFIG = new BreakerSwerveMotorPIDConfig(MODULE_VELOCITY_KP, MODULE_VELOCITY_KI, MODULE_VELOCITY_KD, MODULE_VELOCITY_KF);

      //Module Drive Arbitrary FeedForward
      public static final double FF_STATIC_FRICTION_COEFFICIENT = 0.3;
      public static final double FF_VELOCITY_COEFFICIENT = 2.82;
      public static final BreakerArbitraryFeedforwardProvider MODULE_VELOCITY_FF = new BreakerArbitraryFeedforwardProvider(FF_STATIC_FRICTION_COEFFICIENT, FF_VELOCITY_COEFFICIENT);

      //Module physical constants
      public static final double MAX_ATTAINABLE_MODULE_WHEEL_SPEED = 4.2;
      public static final double DRIVE_MOTOR_GEAR_RATIO_TO_ONE = 8.14;
      public static final double AZIMUTH_MOTOR_GEAR_RATIO_TO_ONE = 1.0;
      public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
      public static final double MODULE_WHEEL_SPEED_DEADBAND = 0.001;
      public static final double AZIMUTH_MOTOR_SUPPLY_CURRENT_LIMIT = 40.0;
      public static final double DRIVE_MOTOR_SUPPLY_CURRENT_LIMIT = 80.0;
      public static final BreakerSwerveModuleConfig MODULE_CONFIG = new BreakerSwerveModuleConfig(
          DRIVE_MOTOR_GEAR_RATIO_TO_ONE, AZIMUTH_MOTOR_GEAR_RATIO_TO_ONE, 
          WHEEL_DIAMETER, 
          MAX_ATTAINABLE_MODULE_WHEEL_SPEED,  
          AZIMUTH_MOTOR_SUPPLY_CURRENT_LIMIT, DRIVE_MOTOR_SUPPLY_CURRENT_LIMIT,
          MODULE_ANGLE_PID_CONFIG, MODULE_VELOCITY_PID_CONFIG, 
          MODULE_VELOCITY_FF
          );

      //X-axis positional PID
      public static final double X_PID_KP = 4.5;
      public static final double X_PID_KI = 0.0;
      public static final double X_PID_KD = 0.0;
      public static final PIDController X_PID = new PIDController(X_PID_KP, X_PID_KI, X_PID_KD);

      //Y-axis positional PID
      public static final double Y_PID_KP = 4.5;
      public static final double Y_PID_KI = 0.0;
      public static final double Y_PID_KD = 0.0;
      public static final PIDController Y_PID = new PIDController(Y_PID_KP, Y_PID_KI, Y_PID_KD);

      //Theta-axis positional PID
      public static final double THETA_PID_KP = 3.5;
      public static final double THETA_PID_KI = 0.0;
      public static final double THETA_PID_KD = 0.0;
      public static final PIDController THETA_PID = new PIDController(THETA_PID_KP, THETA_PID_KI, THETA_PID_KD);

      //Slow mode constants
      public static final double SLOW_MODE_LINEAR_MULTIPLIER = 0.5;
      public static final double SLOW_MODE_TURN_MULTIPLIER = 0.5;

      //Physical Robot Constants
      public static final double MAX_ANGULAR_VEL = ((FL_TRANSLATION.getNorm() * 2.0 * Math.PI) / MAX_ATTAINABLE_MODULE_WHEEL_SPEED) * (2.0 * Math.PI); 
      public static final double MAX_LINEAR_VEL = 4.2;
      public static final double HEADING_COMPENSATION_ANGULAR_VEL_DEADBAND = 0.005;
      public static final double HEADING_COMPENSATION_MIN_ACTIVE_LINEAR_VEL = 0.01;
      public static final BreakerSwerveDriveBaseConfig DRIVE_BASE_CONFIG = new BreakerSwerveDriveBaseConfig(
      MAX_LINEAR_VEL, MAX_ANGULAR_VEL, 
      HEADING_COMPENSATION_ANGULAR_VEL_DEADBAND, HEADING_COMPENSATION_MIN_ACTIVE_LINEAR_VEL, 
      MODULE_WHEEL_SPEED_DEADBAND,
      X_PID, Y_PID, THETA_PID)
      .setSlowModeMultipliers(SLOW_MODE_LINEAR_MULTIPLIER, SLOW_MODE_TURN_MULTIPLIER);

      //heading snap constants
      public static final double HEADING_SNAP_VEL_RAD_PER_SEC = 2*Math.PI;
      public static final double HEADING_SNAP_ACCEL_RAD_PER_SEC_SQ = 10.0;
      public static final double HEADING_SNAP_POSITIONAL_TOLERENCE_RAD = Math.toRadians(1.5);
      public static final double HEADING_SNAP_VELOCITY_TOLERENCE_RAD_PER_SEC = Math.toRadians(9999);
      public static final double HEADING_SNAP_TIMEOUT_SEC = 5.0;
      public static final double HEADING_SNAP_PID_KP = 3.5;
      public static final double HEADING_SNAP_PID_KI = 0.0;
      public static final double HEADING_SNAP_PID_KD = 0.0;
      public static final Constraints HEADING_SNAP_PID_CONSTRAINTS = new Constraints(HEADING_SNAP_VEL_RAD_PER_SEC, HEADING_SNAP_ACCEL_RAD_PER_SEC_SQ);
      public static final ProfiledPIDController HEADING_SNAP_PID = new ProfiledPIDController(HEADING_SNAP_PID_KP,  HEADING_SNAP_PID_KI,  HEADING_SNAP_PID_KD, HEADING_SNAP_PID_CONSTRAINTS);


      //Auto Balance constants
      public static final double BALANCE_ROLL_PID_KP = 0.02;
      public static final double BALANCE_ROLL_PID_KI = 0.0;
      public static final double BALANCE_ROLL_PID_KD = 0.0001;
      public static final PIDController BALANCE_ROLL_PID = new PIDController(BALANCE_ROLL_PID_KP, BALANCE_ROLL_PID_KI, BALANCE_ROLL_PID_KD);
      public static final double BALANCE_ROLL_POSITION_TOLERENCE = 2.0;
      public static final double BALANCE_ROLL_VELOSITY_TOLERENCE = 1.0;

      public static final double BALANCE_PITCH_PID_KP = 0.02;
      public static final double BALANCE_PITCH_PID_KI = 0.0;
      public static final double BALANCE_PITCH_PID_KD = 0.0001;
      public static final PIDController BALANCE_PITCH_PID = new PIDController(BALANCE_PITCH_PID_KP, BALANCE_PITCH_PID_KI, BALANCE_PITCH_PID_KD);
      public static final double BALANCE_PITCH_POSITION_TOLERENCE = 2.0;
      public static final double BALANCE_PITCH_VELOSITY_TOLERENCE = 1.0;

      public static final BreakerSwerveVelocityRequest AUTO_BALANCE_REQUEST = new BreakerSwerveDriveBaseVelocityRequest(new ChassisSpeeds(), SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITHOUT_OFFSET, SlowModeValue.DISABLED, new Translation2d(), false, true);
      public static final double AUTO_BALANCE_TIMEOUT_SEC = 15;

      //MoveToPose command constants
    //   public static final ProfiledPIDController BHDC_X_PID = new  ProfiledPIDController(X_PID_KP, X_PID_KI, X_PID_KD, new Constraints(0.0, 0.0));
    //   public static final ProfiledPIDController BHDC_Y_PID = new  ProfiledPIDController(Y_PID_KP, Y_PID_KI, Y_PID_KD, new Constraints(0.0, 0.0));
      public static final PIDController BHDC_LINEAR_PID = new PIDController(6.5, 0.0, 0.2);
      public static final ProfiledPIDController BHDC_THETA_PID = new ProfiledPIDController(THETA_PID_KP, THETA_PID_KI, THETA_PID_KD, new Constraints(DriveConstants.MAX_ANGULAR_VEL, 4.0));
      //public static final BreakerProfiledHolonomicDriveControler BREAKER_HOLONOMIC_DRIVE_CONTROLLER = new BreakerProfiledHolonomicDriveControler(BHDC_X_PID, BHDC_Y_PID, BHDC_THETA_PID);
      public static final BreakerHolonomicDriveController BREAKER_HOLONOMIC_DRIVE_CONTROLLER = new BreakerHolonomicDriveController(BHDC_LINEAR_PID, BHDC_THETA_PID);

      public static final double MOVE_TO_POSE_TIMEOUT_SEC = 25.0;
      public static final BreakerSwerveVelocityRequest MOVE_TO_POSE_REQUEST = new BreakerSwerveVelocityRequest(new ChassisSpeeds(), SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITHOUT_OFFSET, SlowModeValue.DISABLED, new Translation2d());

      public static final double BHDC_POSE_TOL_X = 0.01;//0.03
      public static final double BHDC_POSE_TOL_Y = 0.01;
      public static final Rotation2d BHDC_POSE_TOL_T = Rotation2d.fromDegrees(1.0);//3.0
      public static final Pose2d BHDC_POSE_TOLERENCE = new Pose2d(BHDC_POSE_TOL_X, BHDC_POSE_TOL_Y, BHDC_POSE_TOL_T);
      public static final double BHDC_VEL_TOL_X = 5.0;//0.005
      public static final double BHDC_VEL_TOL_Y = 5.0;
      public static final double BHDC_VEL_TOL_T = Math.toRadians(25.0);//5.0
      public static final ChassisSpeeds BHDC_VELOCITY_TOLERENCE = new ChassisSpeeds(BHDC_VEL_TOL_X, BHDC_VEL_TOL_Y, BHDC_VEL_TOL_T);

  }

  public static final class ElevatorConstants {
      //General Motor Configs
      public static final int LEFT_MOTOR_ID = 30;
      public static final int RIGHT_MOTOR_ID = 31;
      public static final double SUPPLY_CUR_LIMIT = 30.0;
      public static final double SUPPLY_CUR_LIMIT_TIME = 1.5;

      //Motion Magic Configs
      public static final double MOTION_MAGIC_CRUISE_VEL = 2.0;
      public static final double MOTION_MAGIC_ACCEL = 5.0;
      public static final double MOTION_MAGIC_JERK = 15.0;

      //PIDF Configs
      public static final double PIDF_KP = 15.00;//25
      public static final double PIDF_KI = 0.0;//05
      public static final double PIDF_KD = 0.05;//0
      public static final double PIDF_KS = 0.0;//10
      public static final double PIDF_KV = 0.75;//20

      //Gearing
      public static final double MOTOR_TO_DRUM_GEARING = 5.0; //to one
      public static final double MIN_HEIGHT_GND_REL = 0.2667;
      public static final double MAX_HEIGHT_GND_REL = 1.0557;
      public static final double MAX_HEIGHT_ROBOT_REL = MAX_HEIGHT_GND_REL - MIN_HEIGHT_GND_REL;
      public static final double MIN_HEIGHT_ROBOT_REL = 0;
     
      public static final double MAX_ROT = 17.739;
      public static final double MIN_ROT = 0;
      public static final double MOTOR_ROT_TO_METERS_RATIO =  MAX_ROT / MAX_HEIGHT_ROBOT_REL;

      //Misc
      public static final double BOTTOM_CALIBRATION_DUTY_CYCLE = 0.0;
      public static final double TOP_CALIBRATION_DUTY_CYCLE = 0.1;
      public static final double CALIBRATION_PAHSE_TIMEOUT = 7.5;
      public static final double HIGHT_TOLARENCE = 0.01;

      public static final double ARB_TEST_HEIGHT = 0.6;

      //Command configs
      public static final double MOVE_TO_HEIGHT_COMMAND_TIMEOUT = 5.0;

  }

  public static final class HandConstants {
      public static final int WRIST_ID = 40;
      public static final int ROLLER_ID = 41;
      public static final int WRIST_ENCODER_ID = 42;
      public static final int CONE_BEAM_BRAKE_DIO_PORT = 0;
      public static final int CUBE_BEAM_BRAKE_DIO_PORT = 1;
      public static final boolean BEAM_BRAKE_BROKEN_ON_TRUE = true;

      public static final boolean INVERT_WRIST = false;
      public static final boolean INVERT_ROLLER= false;

      public static final double WRIST_CURRENT_LIMIT = 10.0;
      public static final double WRIST_CURRENT_LIMIT_TIME = 0.5;


      public static final double WRIST_KP = 2.25;
      public static final double WRIST_KI = 0.00;
      public static final double WRIST_KD = 0.0;

      //Feedfoward Constants
      public static final double WRIST_KS = 0.00;
      public static final double WRIST_KG = 0.84;//64
      public static final double WRIST_KV = 0.36;//45
      public static final double WRIST_KA = 0.03;//02

      //Trapezodial Profile Constants
     public static final double WRIST_MAX_VELOCITY_RADS = 3;
     public static final double WRIST_MAX_ACCELERATION_RADS_PER_SEC = 6.0;

      // Roller state constants
      public static final double INTAKE_CONE_DUTY_CYCLE = -0.3;//-0.3
      public static final int INTAKE_CONE_CURENT_LIMIT = 20;

      public static final double INTAKE_CUBE_DUTY_CYCLE = 0.2;//0.2
      public static final int INTAKE_CUBE_CURENT_LIMIT = 20;

      public static final double INTAKE_CONE_GRIP_TIMEOUT = 1.5;
      public static final double INTAKE_CONE_GRIP_DUTY_CYCLE = -0.1;//-0.1
      public static final int INTAKE_CONE_GRIP_CURENT_LIMIT = 7;//5

      public static final double INTAKE_CUBE_GRIP_TIMEOUT = 0.65;
      public static final double INTAKE_CUBE_GRIP_DUTY_CYCLE = 0.07;//0.1
      public static final int INTAKE_CUBE_GRIP_CURENT_LIMIT = 2;//5

      public static final double EXTAKE_CONE_DUTY_CYCLE = 0.75;
      public static final int EXTAKE_CONE_CURENT_LIMIT = 25;

      public static final double EXTAKE_CUBE_DUTY_CYCLE = -0.75;
      public static final int EXTAKE_CUBE_CURENT_LIMIT = 25;

      public static final double CONE_TOF_MAX_DIST = 340.0;
      public static final double CENTERED_CONE_TOF_DISTANCE = 157.0;

      // command constants
      public static final double EJECT_COMMAND_CUTOFF_TRALING_DELAY = 1.0;
      public static final double EJECT_COMMAND_CUTOFF_TIMEOUT = 4.0;

      public static final double WRIST_SET_GOAL_COMMAND_TIMEOUT_SEC = 3.0;

      public static final double ARB_WRIST_TEST_ANG = 60;
  }

  public static class FieldConstants {
      public static final double FIELD_LENGTH_X = 16.4846;

  }

  public static class VisionConstants {
      public static final String FRONT_CAMERA_NAME = "frontCam";
      public static final String LEFT_CAMERA_NAME = "leftCam";
      public static final String RIGHT_CAMERA_NAME = "rightCam";
      public static final String BACK_CAMERA_NAME = "backCam";

      public static final Transform3d FRONT_CAMERA_POSE = new Transform3d(new Translation3d(Units.inchesToMeters(11.371), 0.0, Units.inchesToMeters(6.319)), new Rotation3d(0.0, Math.toRadians(10), Math.toRadians(3.5)));
      public static final Transform3d LEFT_CAMERA_POSE = new Transform3d(new Translation3d(Units.inchesToMeters(2.987), Units.inchesToMeters(12.639), Units.inchesToMeters(20.539)), new Rotation3d(0.0,0.0, Math.toRadians(90.0)));
      public static final Transform3d RIGHT_CAMERA_POSE = new Transform3d(new Translation3d(Units.inchesToMeters(2.987), Units.inchesToMeters(-12.639), Units.inchesToMeters(20.539)), new Rotation3d(0.0,0.0, Math.toRadians(-90.0)));
      public static final Transform3d BACK_CAMERA_POSE =  new Transform3d(new Translation3d(Units.inchesToMeters(-11.371), 0.0, Units.inchesToMeters(6.319)), new Rotation3d(0.0, Math.toRadians(10), Math.toRadians(-176.5)));
      public static final double POSE_FILTER_TRUST_COEF = 5.8;
      public static final double POSE_FILTER_MAX_UNCERTANTY = 0.1;
      public static final double POSE_FILTER_DISTANCE_SCALE_FACTOR = 0.75;
      public static final double POSE_FILTER_MAX_DISTANCE = 2.0;

      public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = exceptionSafeReadApriltags();
      public static final AprilTagFieldLayout APRILTAG_TEST_LAYOUT = new AprilTagFieldLayout(new ArrayList<AprilTag>(), FieldConstants.FIELD_LENGTH_X, 8.001);

      public static final boolean USE_COMP_APRILTAG_LAYOUT = true;

      private static AprilTagFieldLayout exceptionSafeReadApriltags() {
        AprilTagFieldLayout layout;
        if (USE_COMP_APRILTAG_LAYOUT) {
            try {
                layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            } catch (Exception e) {
                layout = new AprilTagFieldLayout(new ArrayList<AprilTag>(), FieldConstants.FIELD_LENGTH_X, 8.001);
            }
        } else {
            layout = APRILTAG_TEST_LAYOUT;
        }
        
        return layout;
      }
  }

  public static final class MiscConstants {
      public static final int IMU_ID = 5;
      public static final double IMU_MOUNT_POSE_PITCH = 0.0;
      public static final double IMU_MOUNT_POSE_YAW = 0.0;
      public static final double IMU_MOUNT_POSE_ROLL = 0.0;
      public static final String CANIVORE_1 = "CANivore_1";
      public static final boolean ALLOW_MANUAL_SCOREING_REALIGNMENT = true;
      public static final String ROBORIO_SN = "03243120";
      // public static final double AUTO_BALANCE_MIN
  }

  public static final class PoseEstimationConstants {
      public static final double ENCODER_ODOMETRY_STANDARD_DEVATION_X_METERS = 0.1;
      public static final double ENCODER_ODOMETRY_STANDARD_DEVATION_Y_METERS = 0.1;
      public static final double ENCODER_ODOMETRY_STANDARD_DEVATION_HEADING_RADIANS = 0.1;
      public static final double[] ENCODER_ODOMETRY_STANDARD_DEVATIONS = new double[] {ENCODER_ODOMETRY_STANDARD_DEVATION_X_METERS, ENCODER_ODOMETRY_STANDARD_DEVATION_Y_METERS, ENCODER_ODOMETRY_STANDARD_DEVATION_HEADING_RADIANS};
      public static final double VISION_ODOMETRY_STANDARD_DEVATION_X_METERS = 0.9;
      public static final double VISION_ODOMETRY_STANDARD_DEVATION_Y_METERS = 0.9;
      public static final double VISION_ODOMETRY_STANDARD_DEVATION_HEADING_RADIANS = 0.9;
      public static final double[] VISION_ODOMETRY_STANDARD_DEVATIONS = new double[] {VISION_ODOMETRY_STANDARD_DEVATION_X_METERS, VISION_ODOMETRY_STANDARD_DEVATION_Y_METERS, VISION_ODOMETRY_STANDARD_DEVATION_HEADING_RADIANS};
  }

  public static final class ScoreingConstants {
      public static final int BLUE_LEFT_NODE_GROUP_CENTRAL_COULMN_ORDINAL = 7;
      public static final int BLUE_CENTER_NODE_GROUP_CENTRAL_COULMN_ORDINAL = 4;
      public static final int BLUE_RIGHT_NODE_GROUP_CENTRAL_COULMN_ORDINAL = 1;
      public static final int RED_LEFT_NODE_GROUP_CENTRAL_COULMN_ORDINAL = 1;
      public static final int RED_CENTER_NODE_GROUP_CENTRAL_COULMN_ORDINAL = 4;
      public static final int RED_RIGHT_NODE_GROUP_CENTRAL_COULMN_ORDINAL = 7;
      public static final Constraints TELEOP_SCOREING_PRE_EXTEND_ALLIGN_LINEAR_CONSTRAINTS = new Constraints(0.075, 0.5);
      public static final Constraints TELEOP_SCOREING_PRE_EXTEND_ALLIGN_ANGULAR_CONSTRAINTS = new Constraints(DriveConstants.MAX_ANGULAR_VEL, 3.0);
      public static final Constraints TELEOP_SCOREING_POST_EXTEND_ALLIGN_CONSTRAINTS = new Constraints(0.75, 0.01);
      public static final Constraints TELEOP_SCOREING_POST_EXTEND_ALLIGN_ANGULAR_CONSTRAINTS = new Constraints(Math.toRadians(5.0), Math.toRadians(10.0));
      public static final double TELEOP_SCOREING_MANUAL_CMD_TIMEOUT = 10.0;

     
    public static final double NODE_ALLIGN_STD_X_OFFSET = 1.87;
      public static final Pose2d C0_BASE_ALLIGNMENT_POSE = new Pose2d(NODE_ALLIGN_STD_X_OFFSET, 0.49, Rotation2d.fromDegrees(180));
      public static final Pose2d C1_BASE_ALLIGNMENT_POSE = new Pose2d(NODE_ALLIGN_STD_X_OFFSET, 1.05, Rotation2d.fromDegrees(180));
      public static final Pose2d C2_BASE_ALLIGNMENT_POSE = new Pose2d(NODE_ALLIGN_STD_X_OFFSET, 1.61, Rotation2d.fromDegrees(180));
      public static final Pose2d C3_BASE_ALLIGNMENT_POSE = new Pose2d(NODE_ALLIGN_STD_X_OFFSET, 2.21, Rotation2d.fromDegrees(180));//C0_BASE_ALLIGNMENT_POSE.plus(new Transform2d(new Translation2d(0.0, INTER_NODE_Y_OFFSET * 3.0), new Rotation2d()));
      public static final Pose2d C4_BASE_ALLIGNMENT_POSE = new Pose2d(NODE_ALLIGN_STD_X_OFFSET, 2.74, Rotation2d.fromDegrees(180));//C0_BASE_ALLIGNMENT_POSE.plus(new Transform2d(new Translation2d(0.0, INTER_NODE_Y_OFFSET * 4.0), new Rotation2d()));
      public static final Pose2d C5_BASE_ALLIGNMENT_POSE = new Pose2d(NODE_ALLIGN_STD_X_OFFSET, 3.29, Rotation2d.fromDegrees(180));//C0_BASE_ALLIGNMENT_POSE.plus(new Transform2d(new Translation2d(0.0, INTER_NODE_Y_OFFSET * 5.0), new Rotation2d()));
      public static final Pose2d C6_BASE_ALLIGNMENT_POSE = new Pose2d(NODE_ALLIGN_STD_X_OFFSET, 3.84, Rotation2d.fromDegrees(180));
      public static final Pose2d C7_BASE_ALLIGNMENT_POSE = new Pose2d(NODE_ALLIGN_STD_X_OFFSET, 4.44, Rotation2d.fromDegrees(180));
      public static final Pose2d C8_BASE_ALLIGNMENT_POSE = new Pose2d(NODE_ALLIGN_STD_X_OFFSET, 4.97, Rotation2d.fromDegrees(180));
      
  }

  public static final class OperatorConstants {
      
      public static final int OPERATOR_PAD_PORT = 1;

  }

  public static final class AutonomousConstants {
    public static final Pose2d NAV_TO_TRAJECTORY_START_TOLERENCE = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(25.0));
    public static final Constraints NAV_TO_TRAJECTORY_START_LINEAR_CONSTRAINTS = new Constraints(2.5, 1.5);
    public static final Constraints NAV_TO_TRAJECTORY_START_ANGULAR_CONSTRAINTS = new Constraints(DriveConstants.MAX_ANGULAR_VEL * 0.75, 1.0);
    public static final HashMap<String, Command> AUTONOMOUS_ACTION_MAP = new HashMap<>();
  }
}
