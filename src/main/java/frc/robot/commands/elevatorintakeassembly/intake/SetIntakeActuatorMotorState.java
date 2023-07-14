// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorintakeassembly.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.ActuatorMotorState;

public class SetIntakeActuatorMotorState extends CommandBase {
  /** Creates a new SetIntakeActuatorMotorState. */
  private Intake intake;
  private ActuatorMotorState motorState; 
  private boolean isInstant;
  private final Timer timer = new Timer();
  public SetIntakeActuatorMotorState(Intake intake, ActuatorMotorState motorState, boolean isInstant) {
   this.intake = intake;
   this.motorState = motorState;
   this.isInstant = isInstant;
    if (!isInstant) {
      addRequirements(intake);
    }
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!isInstant) {
      timer.restart();
    }
    intake.setActuatorMotorState(motorState);
  }

  @Override
  public void execute() {
      if (!isInstant && timer.hasElapsed(IntakeConstants.ACTUATOR_SET_STATE_COMMAND_TIMEOUT_SEC)) {
        this.cancel();
      }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    if (interrupted) {
      BreakerLog.logEvent("setIntakeActuatorMotorState command instance FAILED, command timed out or was cancled");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isInDesiredActuatorState() || isInstant;
  }
}
