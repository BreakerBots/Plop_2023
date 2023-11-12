// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.routines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.commands.auto.actions.BalanceChargeingStation;
import frc.robot.commands.superstructure.SetSuperstructurePositionState;
import frc.robot.commands.superstructure.SuperstructurePositionState;
import frc.robot.commands.superstructure.intake.EjectGamePiece;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NoVisionDemoPath extends SequentialCommandGroup {
  /** Creates a new NoVisionDemoPath. */
  public NoVisionDemoPath(Drive drivetrain, Elevator elevator, Hand hand, BreakerPigeon2 imu) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory demoPath = PathPlanner.loadPath("Demo Path", new PathConstraints(1.75, 3));
    addCommands(
    new InstantCommand(() -> drivetrain.setOdometryPosition(PathPlannerTrajectory.transformTrajectoryForAlliance(demoPath, DriverStation.getAlliance()).getInitialHolonomicPose())),
    new SetSuperstructurePositionState(elevator, hand, SuperstructurePositionState.PLACE_CUBE_HIGH, false),
    new WaitCommand(1.0),
    new EjectGamePiece(hand),
    new SetSuperstructurePositionState(elevator, hand, SuperstructurePositionState.STOW, false),
    drivetrain.followPathCommand(demoPath), // follow path
    new BalanceChargeingStation(drivetrain, imu, false, false));
  }
}
