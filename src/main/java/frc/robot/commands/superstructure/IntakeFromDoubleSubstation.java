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
public class IntakeFromDoubleSubstation extends SetSuperstructureState {
  /** Creates a new IntakeFromGround. */
  public IntakeFromDoubleSubstation(Elevator elevator, Hand hand, boolean verifyIntakeAcutation, GamePieceType gamePieceType) {
    super(
      elevator, 
      hand, 
      gamePieceType.isCube() ? SuperstructurePositionState.PICKUP_DOUBLE_SUBSTATION_CUBE : SuperstructurePositionState.PICKUP_DOUBLE_SUBSTATION_CONE, 
      gamePieceType.isCube() ? IntakeRollerStateRequest.INTAKE_CUBE : IntakeRollerStateRequest.INTAKE_CONE, 
      verifyIntakeAcutation
    );
  }
}
