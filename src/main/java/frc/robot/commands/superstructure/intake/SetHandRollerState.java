// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Hand.ControledGamePieceType;
import frc.robot.subsystems.Hand.RollerState;
import frc.robot.subsystems.Hand.WristGoal.WristGoalType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetHandRollerState extends InstantCommand {
  private Hand hand;
  private IntakeRollerStateRequest stateRequest;
  public SetHandRollerState(Hand hand, IntakeRollerStateRequest stateRequest) {
    this.hand = hand;
    this.stateRequest = stateRequest;
    // addRequirements(hand); // Instant command so should be fine 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ControledGamePieceType controledGamePiece = hand.getControledGamePieceType();
    switch(stateRequest) {


      case EXTAKE:


        BreakerLog.getInstance().logEvent("Intake roller start requested (request: EXTAKE)");
        if (hand.getWristGoalType() != WristGoalType.STOW || hand.getWristGoalType() != WristGoalType.UNKNOWN) {
          switch (controledGamePiece) {
            case CONE:
              hand.rollerExtakeCone();
              BreakerLog.getInstance().logSuperstructureEvent("INTAKE ROLLER EXTAKE REQUEST SUCESSSFULL, ROLLER STARTED EXTAKEING CONE (request: EXTAKE)");
              break;
            case CUBE:
              hand.rollerExtakeCube();
              BreakerLog.getInstance().logSuperstructureEvent("INTAKE ROLLER EXTAKE REQUEST SUCESSSFULL, ROLLER STARTED EXTAKEING CUBE (request: EXTAKE)");
              break;
            case ERROR:
              BreakerLog.getInstance().logEvent("Intake roller extake request FAILED, roller can not enter an extake state while an ERROR beambreak state is detected (request: EXTAKE)");
              break;
            case NONE:
            default:
              BreakerLog.getInstance().logEvent("Intake roller extake request FAILED, roller can not enter an extake state with no controled game piece detected (request: EXTAKE)");
              break;
          }
        } else {
          BreakerLog.getInstance().logEvent("Intake roller extake request FAILED, roller can not enter an extake state while the intake is retracted (request: EXTAKE)");
        }
        break;


      case INTAKE_CONE:


        BreakerLog.getInstance().logEvent("Intake roller start requested (request: INTAKE_CONE)");
        if (hand.getWristGoalType() != WristGoalType.STOW || hand.getWristGoalType() != WristGoalType.UNKNOWN) {
          if (!hand.hasGamePiece()) {
            hand.rollerIntakeCone();
            BreakerLog.getInstance().logSuperstructureEvent("INTAKE ROLLER START REQUEST SUCESSSFULL, ROLLER STARTED INTAKEING (request: INTAKE_CONE)");
          } else {
            BreakerLog.getInstance().logEvent("Intake roller start request FAILED, roller can not enter an intake state while it has a game piece (request: INTAKE_CONE)");
          }
        } else {
          BreakerLog.getInstance().logEvent("Intake roller start request FAILED, roller can not enter an intake state while the intake is retracted (request: INTAKE_CONE)");
        }
        break;


        case INTAKE_CUBE:


        BreakerLog.getInstance().logEvent("Intake roller start requested (request: INTAKE_CUBE)");
        if (hand.getWristGoalType() != WristGoalType.STOW || hand.getWristGoalType() != WristGoalType.UNKNOWN) {
          if (!hand.hasGamePiece()) {
            hand.rollerIntakeCube();
            BreakerLog.getInstance().logSuperstructureEvent("INTAKE ROLLER START REQUEST SUCESSSFULL, ROLLER STARTED INTAKEING  (request: INTAKE_CUBE)");
          } else {
            BreakerLog.getInstance().logEvent("Intake roller start request FAILED, roller can not enter an intake state while it has a game piece (request: INTAKE_CUBE)");
          }
        } else {
          BreakerLog.getInstance().logEvent("Intake roller start request FAILED, roller can not enter an intake state while the intake is retracted (request: INTAKE_CUBE)");
        }
        break;


      case STOP:

  
      default:
        BreakerLog.getInstance().logEvent("Intake roller stop requested (request: STOP)");
        if (hand.getRollerState() != RollerState.NEUTRAL) {
          if (!hand.hasGamePiece()) {
            hand.stopRoller();
            BreakerLog.getInstance().logSuperstructureEvent("INTAKE ROLLER STOP REQUEST SUCESSFULL, INTAKE ROLLER STOPED (request: STOP)");
          } else {
            BreakerLog.getInstance().logEvent("Intake roller stop request FAILED, intake can not be stoped while gripping game piece (request: STOP)");
          }
        } else {
          BreakerLog.getInstance().logEvent("Intake roller stop request FAILED, intake allready stoped (request: STOP)");
        }
        break;
      
    }
  }

  public static enum IntakeRollerStateRequest {
    INTAKE_CONE,
    INTAKE_CUBE,
    EXTAKE,
    STOP
  }
}
