// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.actions.AutoRoutineOdometryResetHeader;
import frc.robot.commands.auto.actions.NavToPathStart;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoHeader extends SequentialCommandGroup {
  /** Creates a new AutoHeader. */
  public AutoHeader(Drive drivetrain, Vision vison, PathPlannerTrajectory path, boolean navToStart) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoRoutineOdometryResetHeader(drivetrain, vison, path, 0.04), //Reset odometry to vision or path init pose if no vision tgts are visiable
      new NavToPathStart(drivetrain, path));
  }
}
