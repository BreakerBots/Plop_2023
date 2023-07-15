// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.routines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.commands.auto.actions.AutoRoutineOdometryResetHeader;
import frc.robot.commands.auto.actions.NavToPathStart;
import frc.robot.commands.drive.BalanceChargeingStation;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DemoPath extends SequentialCommandGroup {
  /** Creates a new DemoPath. */
  public DemoPath(Drive drivetrain, Vision vison, BreakerPigeon2 imu) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory demoPath = PathPlanner.loadPath("Demo Path", new PathConstraints(1.75, 3));
    addCommands(
      new AutoRoutineOdometryResetHeader(drivetrain, vison, demoPath, 0.0), //Reset odometry to vision or path init pose if no vision tgts are visiable
      new NavToPathStart(drivetrain, demoPath), // if robot outside of allowable start pose range comaired to path init pose, navigate to path init pose
      drivetrain.followPathCommand(demoPath), // follow path
      new BalanceChargeingStation(drivetrain, imu, false, false) //balance on chagring statioon for remainig time
    );
  }
}
