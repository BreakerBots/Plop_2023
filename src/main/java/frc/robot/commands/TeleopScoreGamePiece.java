// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GamePieceType;
import frc.robot.Node;
import frc.robot.Node.NodeType;
import frc.robot.OperatorControlPad;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ScoreingConstants;
import frc.robot.commands.drive.MoveToPose;
import frc.robot.commands.rumble.DoublePulseRumble;
import frc.robot.commands.rumble.SinglePulseRumble;
import frc.robot.commands.rumble.TriplePulseRumble;
import frc.robot.commands.superstructure.elevator.ElevatorMoveToHight;
import frc.robot.commands.superstructure.intake.EjectGamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator.ElevatorTargetState;

public class TeleopScoreGamePiece extends CommandBase {
  /** Creates a new ScoreGamePiece. */
  private Node selectedNode;
  private OperatorControlPad operatorControlPad;
  private BreakerXboxController driverController;
  private Drive drivetrain;
  private Elevator elevator;
  private Hand hand;
  private SequentialCommandGroup scoreingSequince;
  public TeleopScoreGamePiece(OperatorControlPad operatorControlPad, BreakerXboxController driverController, Drive drivetrain, Elevator elevator, Hand hand) {
    this.operatorControlPad = operatorControlPad;
    this.driverController = driverController;
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.hand = hand;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BreakerLog.logEvent("TeleopScoreGamePiece instance started");
    Optional<Node> selectedNodeOptional = operatorControlPad.getSelectedScoringNode();

    // checks wheather oporator still has a target node fully selected, the if true, pulls the slecetd node
    if (selectedNodeOptional.isPresent()) {
      selectedNode = selectedNodeOptional.get();
      Optional<GamePieceType> controledGamePiece = hand.getControledGamePieceType().getGamePieceType();

      if (controledGamePiece.isPresent()) {
        if (selectedNode.getType().isGamePieceSupported(controledGamePiece.get())) {

          ElevatorTargetState elevatorTgt = getElevatorTarget();
          Pose2d allignmentPose = selectedNode.getAllignmentPose();
          BreakerLog.logEvent(String.format("TeleopScoreGamePiece instance selected node indentified, scoreing sequince starting (node: %s) (elevator tgt: %s) (allignment pose: %s)", selectedNode.toString(), elevatorTgt.toString(), allignmentPose.toString()));
          new SinglePulseRumble(driverController).schedule();
          Optional<Double> scoringConeOffsetY =  hand.getConeOffset();
          scoreingSequince = 
          new SequentialCommandGroup(
            new ParallelCommandGroup(
              new MoveToPose(selectedNode.getAllignmentPose().plus(new Transform2d(new Translation2d(0.0, (scoringConeOffsetY.isPresent() ? hand.getConeOffset().get() : 0.0)), new Rotation2d())), ScoreingConstants.TELEOP_SCOREING_MOVE_TO_POSE_MAX_LINEAR_VEL, drivetrain), 
              new ElevatorMoveToHight(elevator, getElevatorTarget())
            ),
            new ConditionalCommand(new EjectGamePiece(hand), new InstantCommand(this::cancel), () -> preEjectCheck(elevatorTgt)),
            new InstantCommand(this::postEjectCheck)
            );
        } else {
          BreakerLog.logEvent(String.format("TeleopScoreGamePiece instance FAILED, selected node type is not compatable with currently controled game piece (node: %s) (controled game piece: %s)", selectedNode.toString(), controledGamePiece.get().toString()));
          this.cancel();
        }
      } else {
        BreakerLog.logEvent("TeleopScoreGamePiece instance FAILED, cannot score while robot does not control a game piece");
        this.cancel();
      }
      
    } else {
      BreakerLog.logEvent("TeleopScoreGamePiece instance FAILED, no node selected");
      this.cancel();
    }
  
  }

  @Override
  public void execute() {
      if (operatorControlPad.getScrollClick().getAsBoolean()) {
        BreakerLog.logEvent("TeleopScoreGamePiece instance MANUALY CANCLED, command has ended");
        this.cancel();
      }
  }

  private boolean preEjectCheck(ElevatorTargetState elevatorTarget) {
    boolean check = (elevator.atTargetHeight() && (elevator.getTargetHeightMeters() == elevatorTarget.getTargetHeight())) && BreakerMath.epsilonEqualsPose2d(selectedNode.getAllignmentPose(), drivetrain.getOdometryPoseMeters(), DriveConstants.BHDC_POSE_TOLERENCE);
    if (!check) {
      BreakerLog.logEvent("TeleopScoreGamePiece instance FAILED, game piece eject procedure pre init check failed, elevator or drive not at desired states");
    } else {
      BreakerLog.logEvent("TeleopScoreGamePiece instance game piece eject procedure pre init check PASSED, begining eject");
    }
    return check;

  }

  private void postEjectCheck() {
    if (hand.hasGamePiece()) {
      BreakerLog.logEvent("TeleopScoreGamePiece instance FAILED, post eject check failed, game piece not sucessfully ejected");
      this.cancel();
    } else {
      BreakerLog.logEvent("TeleopScoreGamePiece instance SUCESFULL, post eject check passed, game piece sucessfully ejected");
    }
  }
 
  private ElevatorTargetState getElevatorTarget() {
    switch (selectedNode.getHeight()) {
      case HIGH:
        if (selectedNode.getType() == NodeType.CONE) {
          return ElevatorTargetState.PLACE_CONE_HIGH;
        }
        return ElevatorTargetState.PLACE_CUBE_HIGH;
      case MID:
        if (selectedNode.getType() == NodeType.CONE) {
          return ElevatorTargetState.PLACE_CONE_MID;
        }
        return ElevatorTargetState.PLACE_CUBE_MID;
      case LOW:
      default:
        return ElevatorTargetState.PLACE_HYBRID;
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (Objects.nonNull(scoreingSequince) && scoreingSequince.isScheduled())  {
      scoreingSequince.cancel();
    }
    if (interrupted) {
      new TriplePulseRumble(driverController).schedule();
    } else {
      new DoublePulseRumble(driverController).schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Objects.nonNull(scoreingSequince)) {
      return scoreingSequince.isFinished();
    }
    return false;
  }
}
