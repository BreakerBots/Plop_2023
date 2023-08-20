// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.GamePieceType;
import frc.robot.commands.superstructure.intake.SetHandRollerState;
import frc.robot.commands.superstructure.intake.SetHandRollerState.IntakeRollerStateRequest;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Elevator.ElevatorTargetState;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromSingleSubstation extends ParallelCommandGroup {
  /** Creates a new IntakeFromGround. */
  public IntakeFromSingleSubstation(Elevator elevator, Hand hand, boolean verifyIntakeAcutation, GamePieceType gamePieceType) {
    addCommands(
      new SetSuperstructurePositionState(elevator, hand, gamePieceType.isCube() ? SuperstructurePositionState.PICKUP_SINGLE_SUBSTATION_CUBE : SuperstructurePositionState.PICKUP_SINGLE_SUBSTATION_CONE, verifyIntakeAcutation),
      new SetHandRollerState(hand, gamePieceType.isCube() ? IntakeRollerStateRequest.INTAKE_CUBE : IntakeRollerStateRequest.INTAKE_CONE)
    );
  }
}
