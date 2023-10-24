// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.superstructure.intake.SetHandRollerState;
import frc.robot.commands.superstructure.intake.SetHandRollerState.IntakeRollerStateRequest;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Hand.RollerState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetSuperstructureState extends SequentialCommandGroup {
  /** Creates a new SetSuperstructureState. */
  public SetSuperstructureState(Elevator elevator, Hand hand, SuperstructurePositionState superstructurePositionState, IntakeRollerStateRequest rollerStateRequest, boolean verifyIntakeAcutation) {
    addCommands(
      new SetSuperstructurePositionState(elevator, hand, superstructurePositionState, verifyIntakeAcutation),
      new SetHandRollerState(hand, rollerStateRequest)
    );
  }
}
