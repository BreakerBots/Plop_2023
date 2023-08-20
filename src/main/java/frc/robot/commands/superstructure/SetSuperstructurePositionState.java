// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.superstructure.elevator.ElevatorMoveToHight;
import frc.robot.commands.superstructure.intake.SetHandWristGoal;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Elevator.ElevatorTargetState;
import frc.robot.subsystems.Hand.WristGoal;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetSuperstructurePositionState extends ParallelCommandGroup {
  /** Creates a new SetElevatorIntakeAssemblyState. */
  public SetSuperstructurePositionState(Elevator elevator, Hand hand, SuperstructurePositionState superstructurePositionState, boolean verifyIntakeAcutation) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorMoveToHight(elevator, superstructurePositionState.getElevatorTargetState()),
      new SetHandWristGoal(hand, superstructurePositionState.getWristGoal(), !verifyIntakeAcutation)
    );
  }
}
