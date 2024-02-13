// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.BL_DRIVE_ID;
import static frc.robot.Constants.DriveConstants.BL_ENCODER_ID;
import static frc.robot.Constants.DriveConstants.BL_ENCODER_OFFSET;
import static frc.robot.Constants.DriveConstants.BL_TRANSLATION;
import static frc.robot.Constants.DriveConstants.BL_TURN_ID;
import static frc.robot.Constants.DriveConstants.BR_DRIVE_ID;
import static frc.robot.Constants.DriveConstants.BR_ENCODER_ID;
import static frc.robot.Constants.DriveConstants.BR_ENCODER_OFFSET;
import static frc.robot.Constants.DriveConstants.BR_TRANSLATION;
import static frc.robot.Constants.DriveConstants.BR_TURN_ID;
import static frc.robot.Constants.DriveConstants.DRIVE_BASE_CONFIG;
import static frc.robot.Constants.DriveConstants.FL_DRIVE_ID;
import static frc.robot.Constants.DriveConstants.FL_ENCODER_ID;
import static frc.robot.Constants.DriveConstants.FL_ENCODER_OFFSET;
import static frc.robot.Constants.DriveConstants.FL_TRANSLATION;
import static frc.robot.Constants.DriveConstants.FL_TURN_ID;
import static frc.robot.Constants.DriveConstants.FR_DRIVE_ID;
import static frc.robot.Constants.DriveConstants.FR_ENCODER_ID;
import static frc.robot.Constants.DriveConstants.FR_ENCODER_OFFSET;
import static frc.robot.Constants.DriveConstants.FR_TRANSLATION;
import static frc.robot.Constants.DriveConstants.FR_TURN_ID;
import static frc.robot.Constants.DriveConstants.MODULE_CONFIG;
import static frc.robot.Constants.MiscConstants.CANIVORE_1;

import java.util.Map;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.dashboard.BreakerDashboard;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveBase;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModuleBuilder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveCANcoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerTalonFXSwerveModuleDriveMotor.TalonFXControlOutputUnits;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLogUtil;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;

/** Add your docs here. */
public class Drive extends BreakerSwerveDriveBase {

    private BreakerPigeon2 pigeon;
    //private Vision vision;

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

    private static Timer overCurrentTimer = new Timer();
    private static boolean overCurrentStarted = false;

    private static Map<String, TalonFX> motorMap = Map.of(
        "DRIVE_FL", driveFL,
        "TURN_FL", turnFL,
        "DRIVE_FR", driveFR,
        "TURN_FR", turnFR,
        "DRIVE_BL", driveBL,
        "TURN_BL", turnBL,
        "DRIVE_BR", driveBR,
        "TURN_BR", turnBR
    );
    

    public Drive(BreakerPigeon2 pigeon/*, Vision vision*/) {
        super(DRIVE_BASE_CONFIG, new BreakerSwerveOdometryConfig(), pigeon, frontLeftModule, frontRightModule, backLeftModule, backRightModule);
        //this.vision = vision;
        // Vision2 vision = new Vision2(this);
        // setOdometer(vision);
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

    public void spitOutTemps() {
        // var driveFLTemp = driveFL.getDeviceTemp();
        // var driveFLTurnTemp = turnFL.getDeviceTemp();
        // var driveFRTemp = driveFR.getDeviceTemp();
        // var driveFRTurn = turnFR.getDeviceTemp();
        // var driveBLTemp = driveBL.getDeviceTemp();
        // var driveBLTurnTemp = turnBL.getDeviceTemp();
        // var driveBRTemp = driveBR.getDeviceTemp();
        // var driveBRTurnTemp = turnBR.getDeviceTemp();

        System.out.print(("-=-=-=-=-=-=-=-=-=-=-=-=-=-\n"));
        for (Map.Entry<String, TalonFX> motor : motorMap.entrySet()) {
            System.out.print(motor.getKey() + " TEMP: " + motor.getValue().getDeviceTemp().getValue() + " C\n");
        }
        System.out.println(("-=-=-=-=-=-=-=-=-=-=-=-=-=-"));

        // System.out.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-");
        // System.out.println("DRIVE FL TEMP: " + driveFLTemp);
        // System.out.println("TURN FL TEMP: " + driveFLTurnTemp);
        // System.out.println("DRIVE FR TEMP: " + driveFRTemp);
        // System.out.println("TURN FR TEMP: " + driveFRTurn);
        // System.out.println("DRIVE BL TEMP: " + driveBLTemp);
        // System.out.println("TURN BL TEMP: " + driveBLTurnTemp);
        // System.out.println("DRIVE BR TEMP: " + driveBRTemp);
        // System.out.println("TURN BR TEMP: " + driveBRTurnTemp);
        // System.out.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-");
    }

    // @Override
    // public void setOdometryPosition(Pose2d newPose) {
    //     Optional<Alliance> ally = AllianceManager.getAlliance();
    //     if (ally.get() == Alliance.Red) {
    //         System.out.println(ally.get());
    //         newPose = BreakerMath.mirrorPose(newPose, FIELD_LENGTH_X/2.0, MirrorSymetryAxis2d.Y, MirrorSymetryAxis2d.Y);
    //     } 
    //     super.setOdometryPosition(newPose);
    // }

    // @Override
    // public Pose2d getOdometryPoseMeters() {
    //     Optional<Alliance> ally = AllianceManager.getAlliance();
    //     Pose2d pose = getAbsoluteOdometryPoseMeters();
    //     // if (ally.get() == Alliance.Red) {
            
    //     //         pose =  BreakerMath.mirrorPose(pose, FIELD_LENGTH_X/2.0, MirrorSymetryAxis2d.Y, MirrorSymetryAxis2d.Y);

    //     //     System.out.println(ally.get() == Alliance.Red);
    //     // }
    //     return pose;
    // }

    // public Pose2d getAbsoluteOdometryPoseMeters() {
    //     Pose2d pos = super.getOdometryPoseMeters();
    //     // if(Objects.nonNull(vision) && !Double.isNaN(pos.getX()) && !Double.isNaN(pos.getY()) && !Double.isNaN(pos.getRotation().getRadians()) && vision.isAnyTargetVisable()) {
    //     //     pos = vision.getOdometryPoseMeters();
    //     //     setAbsoluteOdometryPosition(pos);
            
    //     // }
    //     return pos;
    // }

    // public void setAbsoluteOdometryPosition(Pose2d newPose) {
    //     super.setOdometryPosition(newPose);
    // }

    private boolean prevOverCurrentState = false; // lol cry about it

    @Override
    public void periodic() { // brace yourself,  I didn't feel like writing better code :)
        //Pose2d pos = getOdometryPoseMeters();
        // if(!Double.isNaN(pos.getX()) && !Double.isNaN(pos.getY()) && !Double.isNaN(pos.getRotation().getRadians()) && vision.isAnyTargetVisable()) {
        //     setOdometryPosition(vision.getOdometryPoseMeters());
        // }
        //field.setRobotPose(pos);
        
        final double MAX_CURRENT = 70; // womp womp

        StringBuilder warningBuilder = new StringBuilder();
        
        // var driveFLCurrent = driveFL.getSupplyCurrent().getValue();
        // var turnFLCurrent = turnFL.getSupplyCurrent().getValue();

        // var driveFRCurrent = driveFR.getSupplyCurrent().getValue();
        // var turnFRCurrent = turnFR.getSupplyCurrent().getValue();

        // var driveBLCurrent = driveBL.getSupplyCurrent().getValue();
        // var turnBLCurrent = turnBL.getSupplyCurrent().getValue();

        // var driveBRCurrent = driveBR.getSupplyCurrent().getValue();
        // var turnBRCurrent = turnBR.getSupplyCurrent().getValue();
        
        var uhoh = false;
        
        for (var motor : motorMap.entrySet()) {
            var current = motor.getValue().getSupplyCurrent().getValue();
            if (current >= MAX_CURRENT) {
                warningBuilder
                    .append("~")
                    .append(motor.getKey())
                    .append(" CURRENT WARNING: ")
                    .append(current)
                    .append("\n");
                uhoh = true;
            }
        }

        // if (driveFLCurrent >= MAX_CURRENT) {
        //     warningBuilder.append("~DRIVE FL CURRENT WARNING: " + driveFLCurrent + "\n");
        //     uhoh = true;
        // }
        // if (turnFLCurrent >= MAX_CURRENT) {
        //     warningBuilder.append("~TURN FL CURRENT WARNING: " + turnFLCurrent + "\n");
        //     uhoh = true;
        // }
        // if (driveFRCurrent >= MAX_CURRENT) {
        //     warningBuilder.append("~TURN FR CURRENT WARNING: " + driveFRCurrent + "\n");
        //     uhoh = true;
        // }
        // if (turnFRCurrent >= MAX_CURRENT) {
        //     warningBuilder.append("~TURN FR CURRENT WARNING: " + turnFRCurrent + "\n");
        //     uhoh = true;
        // }
        
        // if (driveBLCurrent >= MAX_CURRENT) {
        //     warningBuilder.append("~DRIVE BL CURRENT WARNING: " + driveBLCurrent + "\n");
        //     uhoh = true;
        // }
        // if (turnBLCurrent >= MAX_CURRENT) {
        //     warningBuilder.append("~TURN BL CURRENT WARNING: " + turnBLCurrent + "\n");
        //     uhoh = true;
        // }
        // if (driveBRCurrent >= MAX_CURRENT) {
        //     warningBuilder.append("~DRIVE BR CURRENT WARNING: " + driveBRCurrent + "\n");
        //     uhoh = true;
        // }
        // if (turnBRCurrent >= MAX_CURRENT) {
        //     warningBuilder.append("~TURN BR CURRENT WARNING: " + turnBRCurrent + "\n");
        //     uhoh = true;
        // }

        var overCurrentDuration = overCurrentTimer.get();
        if (overCurrentStarted) {
            
            if (overCurrentDuration > 1.2) {
                for (int i = 0; i < 3; i++) {
                    warningBuilder
                        .append("~!!!!!!!!~ AHHHH OVERCURRENT: ")
                        .append(overCurrentDuration)
                        .append("\n");
                }
            }
            else if (overCurrentDuration > 0.7) {
                warningBuilder
                    .append("~!!!~ OVERCURRENT DURATION: ")
                    .append(overCurrentDuration)
                    .append("\n");
            } else {
                warningBuilder
                    .append("~~OVERCURRENT DURATION: ")
                    .append(overCurrentDuration)
                    .append("\n");
            }
        }

        if (uhoh) {
            if (!overCurrentStarted) {
                overCurrentStarted = true;
                overCurrentTimer.restart();
            }
            System.out.print("~!~ CURRENT WARNINGS ~!~\n");
            System.out.print(warningBuilder.toString());
            System.out.flush();

        } else {
            if (prevOverCurrentState == true) {
                System.out.println(" ~~~!!!~~~ !!! ~~~!!!~~~ ");
                System.out.println("~~~ OVERCURRENT DONE ~~~");
                System.out.println("DURATION: " + overCurrentDuration);
            }
            overCurrentStarted = false;
            overCurrentTimer.reset();
            overCurrentTimer.stop();
        }

        prevOverCurrentState = uhoh;
    }

    @Override
    public void toLog(LogTable table) {
        // TODO Auto-generated method stub
        super.toLog(table);
        table.put("IMUPitchDeg", pigeon.getPitch());
        table.put("IMURollDeg", pigeon.getRoll());
        table.put("hbuhyguyg", BreakerLogUtil.formatPose2dForLog(getOdometryPoseMeters()));
    }
}
