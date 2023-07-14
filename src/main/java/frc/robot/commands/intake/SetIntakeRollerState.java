// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.ActuatorMotorState;
import frc.robot.subsystems.Intake.RollerState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakeRollerState extends InstantCommand {
  private Intake intake;
  private IntakeRollerStateRequest stateRequest;
  public SetIntakeRollerState(Intake intake, IntakeRollerStateRequest stateRequest) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(stateRequest) {
      case EXTAKE:
        BreakerLog.logEvent("Intake roller start requested (request: EXTAKE)");
        if (intake.getActuatorMotorState() == ActuatorMotorState.EXTENDING) {
          intake.extake();
          BreakerLog.logSuperstructureEvent("INTAKE ROLLER EXTAKE REQUEST SUCESSSFULL, ROLLER STARTED EXTAKEING (request: EXTAKE)");
        } else {
          BreakerLog.logEvent("Intake roller extake request FAILED, roller can not enter an extake state while the intake is retracted (request: EXTAKE)");
        }
        break;
      case INTAKE:
        BreakerLog.logEvent("Intake roller start requested (request: INTAKEING)");
        if (intake.getActuatorMotorState() == ActuatorMotorState.EXTENDING) {
          if (!intake.hasGamePiece()) {
            intake.intake();
            BreakerLog.logSuperstructureEvent("INTAKE ROLLER START REQUEST SUCESSSFULL, ROLLER STARTED INTAKEING (request: INTAKEING)");
          } else {
            BreakerLog.logEvent("Intake roller start request FAILED, roller can not enter an intake state while it has a game piece (request: INTAKEING)");
          }
        } else {
          BreakerLog.logEvent("Intake roller start request FAILED, roller can not enter an intake state while the intake is retracted (request: INTAKEING)");
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
    INTAKE,
    EXTAKE,
    STOP
  }
}
