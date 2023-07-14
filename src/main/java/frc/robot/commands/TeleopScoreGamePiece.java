// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Node;
import frc.robot.Node.NodeType;
import frc.robot.OperatorControlPad;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ScoreingConstants;
import frc.robot.commands.drive.MoveToPose;
import frc.robot.commands.elevatorintakeassembly.elevator.ElevatorMoveToHight;
import frc.robot.commands.elevatorintakeassembly.intake.EjectGamePiece;
import frc.robot.commands.rumble.DoublePulseRumble;
import frc.robot.commands.rumble.SinglePulseRumble;
import frc.robot.commands.rumble.TriplePulseRumble;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator.ElevatorTarget;

public class TeleopScoreGamePiece extends CommandBase {
  /** Creates a new ScoreGamePiece. */
  private Node selectedNode;
  private OperatorControlPad operatorControlPad;
  private BreakerXboxController driverController;
  private Drive drivetrain;
  private Elevator elevator;
  private Intake intake;
  private SequentialCommandGroup scoreingSequince;
  public TeleopScoreGamePiece(OperatorControlPad operatorControlPad, BreakerXboxController driverController, Drive drivetrain, Elevator elevator, Intake intake) {
    this.operatorControlPad = operatorControlPad;
    this.driverController = driverController;
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BreakerLog.logEvent("TeleopScoreGamePiece instance started");
    Optional<Node> selectedNodeOptional = operatorControlPad.getSelectedScoringNode();
    if (selectedNodeOptional.isPresent()) {
      selectedNode = selectedNodeOptional.get();
      ElevatorTarget elevatorTgt = getElevatorTarget();
      Pose2d allignmentPose = selectedNode.getAllignmentPose();
      BreakerLog.logEvent(String.format("TeleopScoreGamePiece instance selected node indentified, scoreing sequince starting (node: %s) (elevator tgt: %s) (allignment pose: %s)", selectedNode.toString(), elevatorTgt.toString(), allignmentPose.toString()));
      new SinglePulseRumble(driverController).schedule();
      scoreingSequince = 
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new MoveToPose(selectedNode.getAllignmentPose(), ScoreingConstants.TELEOP_SCOREING_MOVE_TO_POSE_MAX_LINEAR_VEL, drivetrain), 
          new ElevatorMoveToHight(elevator, getElevatorTarget())
        ),
        new ConditionalCommand(new EjectGamePiece(intake), new InstantCommand(this::cancel), () -> preEjectCheck(elevatorTgt)),
        new InstantCommand(this::postEjectCheck)
        );
    } else {
      this.cancel();
      BreakerLog.logEvent("TeleopScoreGamePiece instance FAILED, no node selected");
    }
  
  }

  @Override
  public void execute() {
      if (operatorControlPad.getScrollClick().getAsBoolean()) {
        BreakerLog.logEvent("TeleopScoreGamePiece instance MANUALY CANCLED, command has ended");
        this.cancel();
      }
  }

  private boolean preEjectCheck(ElevatorTarget elevatorTarget) {
    boolean check = (elevator.atTargetHeight() && (elevator.getTargetHeightMeters() == elevatorTarget.getTargetHeight())) && BreakerMath.epsilonEqualsPose2d(selectedNode.getAllignmentPose(), drivetrain.getOdometryPoseMeters(), DriveConstants.BHDC_POSE_TOLERENCE);
    if (!check) {
      BreakerLog.logEvent("TeleopScoreGamePiece instance FAILED, game piece eject procedure pre init check failed, elevator or drive not at desired states");
    } else {
      BreakerLog.logEvent("TeleopScoreGamePiece instance game piece eject procedure pre init check PASSED, begining eject");
    }
    return check;

  }

  private void postEjectCheck() {
    if (intake.hasGamePiece()) {
      BreakerLog.logEvent("TeleopScoreGamePiece instance FAILED, post eject check failed, game piece not sucessfully ejected");
      this.cancel();
    } else {
      BreakerLog.logEvent("TeleopScoreGamePiece instance SUCESFULL, post eject check passed, game piece sucessfully ejected");
    }
  }
 
  private ElevatorTarget getElevatorTarget() {
    switch (selectedNode.getHeight()) {
      case HIGH:
        if (selectedNode.getType() == NodeType.CONE) {
          return ElevatorTarget.PLACE_CONE_HIGH;
        }
        return ElevatorTarget.PLACE_CUBE_HIGH;
      case MID:
        if (selectedNode.getType() == NodeType.CONE) {
          return ElevatorTarget.PLACE_CONE_MID;
        }
        return ElevatorTarget.PLACE_CUBE_MID;
      case LOW:
      default:
        return ElevatorTarget.PLACE_HYBRID;
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
