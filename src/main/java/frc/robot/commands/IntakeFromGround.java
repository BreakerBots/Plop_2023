// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.GamePieceType2;
import frc.robot.commands.intake.SetIntakeRollerState;
import frc.robot.commands.intake.SetIntakeRollerState.IntakeRollerStateRequest;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorTarget;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.ActuatorMotorState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromGround extends ParallelCommandGroup {
  /** Creates a new IntakeFromGround. */
  public IntakeFromGround(Elevator elevator, Intake intake, boolean verifyIntakeAcutation, GamePieceType2 gamePieceType) {
    addCommands(
      new SetElevatorIntakeAssemblyState(elevator, intake, gamePieceType.isCube() ? ElevatorTarget.PICKUP_GROUND_CUBE : ElevatorTarget.PICKUP_GROUND_CONE, ActuatorMotorState.EXTENDING, verifyIntakeAcutation),
      new SetIntakeRollerState(intake, IntakeRollerStateRequest.INTAKE)
    );
  }
}
