// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Elevator.ElevatorTarget;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorMoveToHight extends CommandBase {
  /** Creates a new ElevatorMoveToHight. */
  private Elevator elevator;
  private double targetHightMeters;
  private final Timer timer = new Timer();
  public ElevatorMoveToHight(Elevator elevator, double targetHightMeters) {
    this.elevator = elevator;
    this.targetHightMeters = targetHightMeters;
    addRequirements(elevator);
  }

  public ElevatorMoveToHight(Elevator elevator, ElevatorTarget target) {
    this(elevator, target.getTargetHeight());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BreakerLog.logSuperstructureEvent(String.format("AN ElevatorMoveToHight COMMAND INSTANCE HAS BEEN INITALZED (tgt: %.2f meters)", targetHightMeters));
    elevator.setTarget(targetHightMeters);
    timer.restart();
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(ElevatorConstants.MOVE_TO_HEIGHT_COMMAND_TIMEOUT)) {
      this.cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
      if (interrupted) {
        BreakerLog.logSuperstructureEvent(String.format("AN ElevatorMoveToHight COMMAND INSTANCE TIMED OUT OR WAS INTERRUPTED (tgt: %.2f meters)", targetHightMeters));
      }
      BreakerLog.logSuperstructureEvent(String.format("AN ElevatorMoveToHight COMMAND INSTANCE ENDED NORMALY AND WAS SUCCESSFULL (tgt: %.2f meters)", targetHightMeters));
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.atTargetHeight();
  }
}
