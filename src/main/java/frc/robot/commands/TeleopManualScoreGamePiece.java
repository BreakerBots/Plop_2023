// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.GamePieceType;
import frc.robot.OperatorControlPad;
import frc.robot.BreakerLib.driverstation.gamepad.BreakerGamepadTimedRumbleCommand;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.Constants.ScoreingConstants;
import frc.robot.Node.NodeHeight;
import frc.robot.commands.rumble.DoublePulseRumble;
import frc.robot.commands.rumble.SinglePulseRumble;
import frc.robot.commands.rumble.TriplePulseRumble;
import frc.robot.commands.superstructure.SetSuperstructurePositionState;
import frc.robot.commands.superstructure.SuperstructurePositionState;
import frc.robot.commands.superstructure.intake.EjectGamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Hand.ControledGamePieceType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopManualScoreGamePiece extends ParallelRaceGroup {
  private BreakerXboxController driverController;
  public TeleopManualScoreGamePiece(NodeHeight targetNodeHeight, BreakerXboxController driverController, Elevator elevator, Hand hand) {
    this.driverController = driverController;
    addCommands(
      new WaitUntilCommand(ScoreingConstants.TELEOP_SCOREING_MANUAL_CMD_TIMEOUT)
      .andThen(
        new TriplePulseRumble(driverController)
        .alongWith(new InstantCommand(() -> cancleCmd("TeleopManualScoreGamePiece instance FAILED, timed out")))),
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new SinglePulseRumble(driverController),
          new ParallelDeadlineGroup(
            new WaitUntilCommand(driverController.getButtonB()),
            new SequentialCommandGroup(
              new InstantCommand(() -> preEjectCheck(hand)),
              new SetElevatorToScoreingPosition(elevator, hand, targetNodeHeight),
              new BreakerGamepadTimedRumbleCommand(driverController, 3.0, 1.0, 1.0)
            )
          )
        ),
        new EjectGamePiece(hand),
        new ConditionalCommand(new InstantCommand(() -> cmdEndMsgFail()), new InstantCommand(() -> cmdEndMsgSuccess()), hand::hasGamePiece)
      )
    );
  }

  private void preEjectCheck(Hand hand) {
    if (!hand.hasGamePiece()) {
      cancleCmd("TeleopManualScoreGamePiece instance FAILED, could not detect controled game piece");
    }
  }

  private void cancleCmd(String msg) {
    new TriplePulseRumble(driverController).schedule();
    BreakerLog.getInstance().logEvent(msg);
    this.cancel();
  }

  private void cmdEndMsgFail() {
    new DoublePulseRumble(driverController).schedule();
    BreakerLog.getInstance().logEvent("TeleopManualScoreGamePiece instance FAILED, post eject check failed, game piece not sucessfully ejected");
  }

  private void cmdEndMsgSuccess() {
    new DoublePulseRumble(driverController).schedule();
    BreakerLog.getInstance().logEvent("TeleopManualScoreGamePiece instance SUCESFULL, post eject check passed, game piece sucessfully ejected");
  }

  private static class SetElevatorToScoreingPosition extends CommandBase {
    private SetSuperstructurePositionState sspt;
    private Elevator elevator;
    private Hand hand;
    private NodeHeight targetNodeHeight;
    public SetElevatorToScoreingPosition(Elevator elevator, Hand hand, NodeHeight targetNodeHeight) {
      this.elevator = elevator;
      this.hand = hand;
      this.targetNodeHeight = targetNodeHeight;
    }

    @Override
    public void initialize() {
      sspt = new SetSuperstructurePositionState(elevator, hand, getTargetState(), false);
      sspt.schedule();
    }

    private SuperstructurePositionState getTargetState() {
      Optional<GamePieceType> gpt = hand.getControledGamePieceType().getGamePieceType();
     if (gpt.isPresent()) {
        switch (targetNodeHeight) {
          case HIGH:
            if (gpt.get() == GamePieceType.CONE) {
              return SuperstructurePositionState.PLACE_CONE_HIGH;
            }
            return SuperstructurePositionState.PLACE_CUBE_HIGH;
          case MID:
            if (gpt.get() == GamePieceType.CONE) {
              return SuperstructurePositionState.PLACE_CONE_MID;
            }
            return SuperstructurePositionState.PLACE_CUBE_MID;
          case LOW:
          default:
            return SuperstructurePositionState.PLACE_HYBRID;
        }
      } else {
        return SuperstructurePositionState.STOW;
      }
    }

    @Override
    public boolean isFinished() {
        if (Objects.nonNull(sspt)) {
          return sspt.isFinished();
        }
        return true;
    }
  }
}
