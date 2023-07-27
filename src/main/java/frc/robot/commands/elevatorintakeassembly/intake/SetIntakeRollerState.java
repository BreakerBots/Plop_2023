// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorintakeassembly.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.ActuatorMotorState;
import frc.robot.subsystems.Intake.ControledGamePieceType;
import frc.robot.subsystems.Intake.RollerState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakeRollerState extends InstantCommand {
  private Intake intake;
  private IntakeRollerStateRequest stateRequest;
  public SetIntakeRollerState(Intake intake, IntakeRollerStateRequest stateRequest) {
    this.intake = intake;
    this.stateRequest = stateRequest;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ControledGamePieceType controledGamePiece = intake.getControledGamePieceType();
    switch(stateRequest) {


      case EXTAKE:


        BreakerLog.logEvent("Intake roller start requested (request: EXTAKE)");
        if (intake.getActuatorMotorState() == ActuatorMotorState.EXTENDING) {
          switch (controledGamePiece) {
            case CONE:
              intake.extakeCone();
              BreakerLog.logSuperstructureEvent("INTAKE ROLLER EXTAKE REQUEST SUCESSSFULL, ROLLER STARTED EXTAKEING CONE (request: EXTAKE)");
              break;
            case CUBE:
              intake.extakeCube();
              BreakerLog.logSuperstructureEvent("INTAKE ROLLER EXTAKE REQUEST SUCESSSFULL, ROLLER STARTED EXTAKEING CUBE (request: EXTAKE)");
              break;
            case ERROR:
              BreakerLog.logEvent("Intake roller extake request FAILED, roller can not enter an extake state while an ERROR beambreak state is detected (request: EXTAKE)");
              break;
            case NONE:
            default:
              BreakerLog.logEvent("Intake roller extake request FAILED, roller can not enter an extake state with no controled game piece detected (request: EXTAKE)");
              break;
          }
        } else {
          BreakerLog.logEvent("Intake roller extake request FAILED, roller can not enter an extake state while the intake is retracted (request: EXTAKE)");
        }
        break;


      case INTAKE_CONE:


        BreakerLog.logEvent("Intake roller start requested (request: INTAKE_CONE)");
        if (intake.getActuatorMotorState() == ActuatorMotorState.EXTENDING) {
          if (!intake.hasGamePiece()) {
            intake.intakeCone();
            BreakerLog.logSuperstructureEvent("INTAKE ROLLER START REQUEST SUCESSSFULL, ROLLER STARTED INTAKEING (request: INTAKE_CONE)");
          } else {
            BreakerLog.logEvent("Intake roller start request FAILED, roller can not enter an intake state while it has a game piece (request: INTAKE_CONE)");
          }
        } else {
          BreakerLog.logEvent("Intake roller start request FAILED, roller can not enter an intake state while the intake is retracted (request: INTAKE_CONE)");
        }
        break;


        case INTAKE_CUBE:


        BreakerLog.logEvent("Intake roller start requested (request: INTAKE_CUBE)");
        if (intake.getActuatorMotorState() == ActuatorMotorState.EXTENDING) {
          if (!intake.hasGamePiece()) {
            intake.intakeCube();
            BreakerLog.logSuperstructureEvent("INTAKE ROLLER START REQUEST SUCESSSFULL, ROLLER STARTED INTAKEING  (request: INTAKE_CUBE)");
          } else {
            BreakerLog.logEvent("Intake roller start request FAILED, roller can not enter an intake state while it has a game piece (request: INTAKE_CUBE)");
          }
        } else {
          BreakerLog.logEvent("Intake roller start request FAILED, roller can not enter an intake state while the intake is retracted (request: INTAKE_CUBE)");
        }
        break;


      case STOP:

  
      default:
        BreakerLog.logEvent("Intake roller stop requested (request: STOP)");
        if (intake.getRollerState() != RollerState.NEUTRAL) {
          if (!intake.hasGamePiece()) {
            intake.stopRoller();
            BreakerLog.logSuperstructureEvent("INTAKE ROLLER STOP REQUEST SUCESSFULL, INTAKE ROLLER STOPED (request: STOP)");
          } else {
            BreakerLog.logEvent("Intake roller stop request FAILED, intake can not be stoped while gripping game piece (request: STOP)");
          }
        } else {
          BreakerLog.logEvent("Intake roller stop request FAILED, intake allready stoped (request: STOP)");
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
