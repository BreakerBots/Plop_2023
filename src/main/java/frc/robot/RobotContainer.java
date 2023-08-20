// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoPath;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.dashboard.BreakerDashboard;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController;
import frc.robot.BreakerLib.util.robot.BreakerRobotConfig;
import frc.robot.BreakerLib.util.robot.BreakerRobotManager;
import frc.robot.BreakerLib.util.robot.BreakerRobotStartConfig;
import frc.robot.commands.TeleopScoreGamePiece;
import frc.robot.commands.auto.routines.DemoPath;
import frc.robot.commands.drive.TeleopBalanceChargingStation;
import frc.robot.commands.drive.TeleopSnapDriveToCardinalHeading;
import frc.robot.commands.drive.TeleopSnapDriveToCardinalHeading.SwerveCardinal;
import frc.robot.commands.superstructure.IntakeFromDoubleSubstation;
import frc.robot.commands.superstructure.IntakeFromGround;
import frc.robot.commands.superstructure.IntakeFromSingleSubstation;
import frc.robot.commands.superstructure.StowElevatorIntakeAssembly;
import frc.robot.commands.superstructure.elevator.ManuallyControlElevator;
import frc.robot.commands.superstructure.intake.EjectGamePiece;
import frc.robot.commands.superstructure.intake.SetHandRollerState;
import frc.robot.commands.superstructure.intake.SetHandRollerState.IntakeRollerStateRequest;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import static frc.robot.Constants.AutonomousConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private static final BreakerXboxController driverControllerSys = new BreakerXboxController(0);
    private static final OperatorControlPad operatorControlPadSys = new OperatorControlPad(OperatorConstants.OPERATOR_PAD_PORT);

    private static final Vision visionSys = new Vision();
    private static final BreakerPigeon2 imuSys = new BreakerPigeon2(MiscConstants.IMU_ID, MiscConstants.CANIVORE_1);

    private static final Drive drivetrainSys = new Drive(imuSys, visionSys);
    private static final Elevator elevatorSys = new Elevator();
    private static final Hand handSys = new Hand();
  
    private static final BreakerTeleopSwerveDriveController teleopDriveController = new BreakerTeleopSwerveDriveController(drivetrainSys, driverControllerSys);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrainSys.setDefaultCommand(teleopDriveController);
    
    configureBindings();
    configureAutonomousActionMap();
    configureIMU();
    configureRobotManager();
  }

  public void configureIMU() {
    MountPoseConfigs mountPoseConfigs = new MountPoseConfigs();
    mountPoseConfigs.MountPosePitch = MiscConstants.IMU_MOUNT_POSE_PITCH;
    mountPoseConfigs.MountPoseYaw = MiscConstants.IMU_MOUNT_POSE_YAW;
    mountPoseConfigs.MountPoseRoll = MiscConstants.IMU_MOUNT_POSE_ROLL;
    imuSys.getConfigurator().apply(mountPoseConfigs);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //drive controls
    driverControllerSys.getDPad().getUp().onTrue(new TeleopSnapDriveToCardinalHeading(SwerveCardinal.FRONT, drivetrainSys, teleopDriveController));
    driverControllerSys.getDPad().getLeft().onTrue(new TeleopSnapDriveToCardinalHeading(SwerveCardinal.LEFT, drivetrainSys, teleopDriveController));
    driverControllerSys.getDPad().getRight().onTrue(new TeleopSnapDriveToCardinalHeading(SwerveCardinal.RIGHT, drivetrainSys, teleopDriveController));
    driverControllerSys.getDPad().getDown().onTrue(new TeleopSnapDriveToCardinalHeading(SwerveCardinal.BACK, drivetrainSys, teleopDriveController));

    //scoreing controls
    operatorControlPadSys.getScoringCommandRequestTrigger().onTrue(new TeleopScoreGamePiece(operatorControlPadSys, driverControllerSys ,drivetrainSys, elevatorSys, handSys));

    //stow elevator (driver controls: LB / RB = stow) (operator controls: 20 = stow)
    driverControllerSys.getLeftBumper()
    .or(driverControllerSys.getRightBumper())
    .or(operatorControlPadSys.getElevatorStowButton())
    .onTrue(new StowElevatorIntakeAssembly(elevatorSys, handSys, true));

    //intake from ground, (driver controls: X = cube, Y = cone) (operator controls: 14 = cube, 15 = cone)
    driverControllerSys.getButtonY().or(operatorControlPadSys.getIntakeGroundConeButton()).onTrue(new IntakeFromGround(elevatorSys, handSys, true, GamePieceType.CONE));
    driverControllerSys.getButtonX().or(operatorControlPadSys.getIntakeGroundCubeButton()).onTrue(new IntakeFromGround(elevatorSys, handSys, true, GamePieceType.CUBE));

    //intake from single sub, (operator controls: 9 = cube, 10 = cone)
    operatorControlPadSys.getIntakeSingleSubstationConeButton().onTrue(new IntakeFromSingleSubstation(elevatorSys, handSys, true, GamePieceType.CONE));
    operatorControlPadSys.getIntakeSingleSubstationCubeButton().onTrue(new IntakeFromSingleSubstation(elevatorSys, handSys, true, GamePieceType.CUBE));

    //intake from double sub, (operator controls: 4 = cube, 5 = cone)
    operatorControlPadSys.getIntakeDoubleSubstationConeButton().onTrue(new IntakeFromDoubleSubstation(elevatorSys, handSys, true, GamePieceType.CONE));
    operatorControlPadSys.getIntakeDoubleSubstationCubeButton().onTrue(new IntakeFromDoubleSubstation(elevatorSys, handSys, true, GamePieceType.CUBE));

    //intake roller controls, (driver controls: A = intake, B = stop) (operator controls: 19 = extake)
    //driverControllerSys.getButtonA().onTrue(new SetIntakeRollerState(intakeSys, IntakeRollerStateRequest.INTAKE));
    //driverControllerSys.getButtonB().onTrue(new SetIntakeRollerState(intakeSys, IntakeRollerStateRequest.STOP));
    operatorControlPadSys.getRollerExtakeButton().onTrue(new SetHandRollerState(handSys, IntakeRollerStateRequest.EXTAKE));

    //game piece eject, (operator controls: fire button)
    operatorControlPadSys.getEjectGamePieceButton().onTrue(new EjectGamePiece(handSys));

    //teleop autobalance
    driverControllerSys.getStartButton().toggleOnTrue(new TeleopBalanceChargingStation(drivetrainSys, imuSys, teleopDriveController, driverControllerSys, false, false));
  }

  private void configureAutonomousActionMap() {
    AUTONOMOUS_ACTION_MAP.put("EJECT_GAME_PIECE", new EjectGamePiece(handSys));
    AUTONOMOUS_ACTION_MAP.put("INTAKE_CONE_FROM_GROUND", new IntakeFromGround(elevatorSys, handSys, false, GamePieceType.CONE));
    AUTONOMOUS_ACTION_MAP.put("INTAKE_CUBE_FROM_GROUND", new IntakeFromGround(elevatorSys, handSys, false, GamePieceType.CUBE));
    AUTONOMOUS_ACTION_MAP.put("STOW", new StowElevatorIntakeAssembly(elevatorSys, handSys, false));
    AUTONOMOUS_ACTION_MAP.put("", teleopDriveController);
    
  }

  private void configureRobotManager() {
    BreakerRobotConfig robotConfig = new BreakerRobotConfig(new BreakerRobotStartConfig(5104, "BreakerBots",
    "Plop", 2023, "v1", "Roman Abrahamson"));
    robotConfig.setAutoPaths(new BreakerAutoPath("Demo Path", new DemoPath(drivetrainSys, visionSys, imuSys)));
    BreakerRobotManager.setup(drivetrainSys, robotConfig);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return BreakerRobotManager.getSelectedAutoPath();
  }
}
