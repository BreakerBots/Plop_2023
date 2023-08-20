// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.superstructure.intake.SetHandRollerState.IntakeRollerStateRequest;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Hand.WristGoal;
import frc.robot.subsystems.Hand.WristGoal.WristGoalType;

public class EjectGamePiece extends SequentialCommandGroup {
  /** Creates a new EjectGamePiece. */
  public EjectGamePiece(Hand hand) {
    addCommands(
      new InstantCommand(() -> {
       BreakerLog.logSuperstructureEvent("GAME PIECE EJECTION PROCEDURE STARTED");
        if (hand.getWristGoalType() == WristGoalType.STOW || hand.getWristGoalType() == WristGoalType.UNKNOWN) {
          BreakerLog.logEvent("GAME PIECE EJECTION PROCEDURE FAILED, WRIST NOT IN EJECTABLE POSITION");
          this.cancel();
        }
      }
      ),
      new SetHandRollerState(hand, IntakeRollerStateRequest.EXTAKE),
      new ParallelRaceGroup(new SequentialCommandGroup(new WaitUntilCommand(() -> !hand.hasGamePiece()), new WaitCommand(IntakeConstants.EJECT_COMMAND_CUTOFF_TRALING_DELAY)), new WaitCommand(IntakeConstants.EJECT_COMMAND_CUTOFF_TIMEOUT)),
      new SetHandRollerState(hand, IntakeRollerStateRequest.STOP),
      new InstantCommand(() -> {if (hand.hasGamePiece()) { BreakerLog.logSuperstructureEvent("GAME PIECE EJECTION PROCEDURE FAILED, GAME PIECE NOT PROPERLY EJECTED, COMMMAND TIMED OUT"); } else { BreakerLog.logSuperstructureEvent("GAME PIECE EJECTION PROCEDURE SUCESSFULL, GAME PIECE EJECTED");}})
    );
  }
}
