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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.GamePieceType;
import frc.robot.Node;
import frc.robot.Node.NodeHeight;
import frc.robot.Node.NodeType;
import frc.robot.OperatorControlPad;
import frc.robot.RobotContainer;
import frc.robot.BreakerLib.driverstation.gamepad.BreakerGamepadTimedRumbleCommand;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ScoreingConstants;
import frc.robot.commands.drive.MoveToPose;
import frc.robot.commands.rumble.DoublePulseRumble;
import frc.robot.commands.rumble.SinglePulseRumble;
import frc.robot.commands.rumble.TriplePulseRumble;
import frc.robot.commands.superstructure.SetSuperstructurePositionState;
import frc.robot.commands.superstructure.SuperstructurePositionState;
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
    BreakerLog.getInstance().logEvent("TeleopScoreGamePiece instance started");
    Optional<Node> selectedNodeOptional = operatorControlPad.getSelectedScoringNode();

    // checks wheather oporator still has a target node fully selected, the if true, pulls the slecetd node
    if (selectedNodeOptional.isPresent()) {
      selectedNode = selectedNodeOptional.get();
      System.out.println(selectedNodeOptional.get().toString()); 
      Optional<GamePieceType> controledGamePiece = hand.getControledGamePieceType().getGamePieceType();

      // if (controledGamePiece.isPresent()) {
      //   if (selectedNode.getType().isGamePieceSupported(controledGamePiece.get())) {

          SuperstructurePositionState superTgt = getSuperstructureTarget();
          Optional<Double> scoringConeOffsetY =  hand.getConeOffset();
          Pose2d allignmentPose = selectedNode.getAllignmentPose().plus(new Transform2d(new Translation2d(0.0, (scoringConeOffsetY.isPresent() ? hand.getConeOffset().get() : 0.0)), new Rotation2d()));
         // Pose2d extensionPose = selectedNode.getExtensionAllignmentPose().plus(new Transform2d(new Translation2d(0.0, (scoringConeOffsetY.isPresent() ? hand.getConeOffset().get() : 0.0)), new Rotation2d()));
          BreakerLog.getInstance().logEvent(String.format("TeleopScoreGamePiece instance selected node indentified, scoreing sequince starting (node: %s) (superstructure tgt: %s) (allignment pose: %s)", selectedNode.toString(), superTgt.toString(), allignmentPose.toString()));
          new SinglePulseRumble(driverController).schedule();
          //if (selectedNode.getHeight() != NodeHeight.LOW) {
            // scoreingSequince = 
            //   new SequentialCommandGroup( 
            //     new MoveToPose(allignmentPose, drivetrain, ScoreingConstants.TELEOP_SCOREING_PRE_EXTEND_ALLIGN_LINEAR_CONSTRAINTS, ScoreingConstants.TELEOP_SCOREING_POST_EXTEND_ALLIGN_ANGULAR_CONSTRAINTS),
            //     new SetSuperstructurePositionState(elevator, hand, superTgt, false),
            //     new MoveToPose(allignmentPose, drivetrain, ScoreingConstants.TELEOP_SCOREING_POST_EXTEND_ALLIGN_CONSTRAINTS, ScoreingConstants.TELEOP_SCOREING_POST_EXTEND_ALLIGN_ANGULAR_CONSTRAINTS),
            //     //new ConditionalCommand(new EjectGamePiece(hand), new InstantCommand(this::cancel), () -> preEjectCheck(superTgt)),
            //     new EjectGamePiece(hand),
            //     new InstantCommand(this::postEjectCheck)
            //   );
          //} else {
            scoreingSequince = 
              new SequentialCommandGroup(
                //new InstantCommand(() -> new SetSuperstructurePositionState(elevator, hand, SuperstructurePositionState.STOW, false).schedule());
                new MoveToPose(allignmentPose, drivetrain,  1.75).asProxy(),
                new SetSuperstructurePositionState(elevator, hand, superTgt, false),
                new InstantCommand(() -> driverController.setMixedRumble(6.0, 6.0)),
                new WaitUntilCommand(driverController.getButtonB()),
                new InstantCommand(() -> driverController.setMixedRumble(0.0, 0.0)),
                new EjectGamePiece(hand),
                new InstantCommand(() -> new SetSuperstructurePositionState(elevator, hand, SuperstructurePositionState.STOW, false).schedule()),
                new InstantCommand(this::postEjectCheck)
              );
          }
        // } else {
        //   BreakerLog.getInstance().logEvent(String.format("TeleopScoreGamePiece instance FAILED, selected node type is not compatable with currently controled game piece (node: %s) (controled game piece: %s)", selectedNode.toString(), controledGamePiece.get().toString()));
        //   this.cancel();
        // }
    //   } else {
    //     BreakerLog.getInstance().logEvent("TeleopScoreGamePiece instance FAILED, cannot score while robot does not control a game piece");
    //     this.cancel();
    //   }
      
    // } else {
    //   BreakerLog.getInstance().logEvent("TeleopScoreGamePiece instance FAILED, no node selected");
    //   this.cancel();
    // }

    if (Objects.nonNull(scoreingSequince)) {
      scoreingSequince.schedule();
    }

  
  }

  @Override
  public void execute() {
      if (RobotContainer.globalCancel()) {
        BreakerLog.getInstance().logEvent("TeleopScoreGamePiece instance MANUALY CANCLED, command has ended");
        this.cancel();
      }
  }

  // private boolean preEjectCheck(SuperstructurePositionState superTarget) {
  //   boolean check = (elevator.atTargetHeight() && (elevator.getTargetHeightMeters() == superTarget.getElevatorTargetState().getTargetHeight())) && (hand.atWristGoal() && hand.getWristGoalAngle() == superTarget.getWristGoal().getGoalAngle()) && BreakerMath.epsilonEqualsPose2d(selectedNode.getAllignmentPose(), drivetrain.getOdometryPoseMeters(), DriveConstants.BHDC_POSE_TOLERENCE);
  //   if (!check) {
  //     BreakerLog.getInstance().logEvent("TeleopScoreGamePiece instance FAILED, game piece eject procedure pre init check failed, elevator or drive not at desired states");
  //   } else {
  //     BreakerLog.getInstance().logEvent("TeleopScoreGamePiece instance game piece eject procedure pre init check PASSED, begining eject");
  //   }
  //   return check;

  // }

  private void postEjectCheck() {
    if (hand.hasGamePiece()) {
      BreakerLog.getInstance().logEvent("TeleopScoreGamePiece instance FAILED, post eject check failed, game piece not sucessfully ejected");
      this.cancel();
    } else {
      BreakerLog.getInstance().logEvent("TeleopScoreGamePiece instance SUCESFULL, post eject check passed, game piece sucessfully ejected");
    }
  }
 
  private SuperstructurePositionState getSuperstructureTarget() {
    switch (selectedNode.getHeight()) {
      case HIGH:
        if (selectedNode.getType() == NodeType.CONE) {
          return SuperstructurePositionState.PLACE_CONE_HIGH;
        }
        return SuperstructurePositionState.PLACE_CUBE_HIGH;
      case MID:
        if (selectedNode.getType() == NodeType.CONE) {
          return SuperstructurePositionState.PLACE_CONE_MID;
        }
        return SuperstructurePositionState.PLACE_CUBE_MID;
      case LOW:
      default:
        return SuperstructurePositionState.PLACE_HYBRID;
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
