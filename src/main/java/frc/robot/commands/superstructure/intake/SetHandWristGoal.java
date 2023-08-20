// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Hand.WristGoal;
import frc.robot.BreakerLib.util.logging.BreakerLog;


public class SetHandWristGoal extends CommandBase {
  /** Creates a new SetIntakeActuatorMotorState. */
  private Hand hand;
  private WristGoal wristGoal; 
  private boolean isInstant;
  private final Timer timer = new Timer();
  public SetHandWristGoal(Hand hand, WristGoal wristGoal, boolean isInstant) {
   this.hand = hand;
   this.wristGoal = wristGoal;
   this.isInstant = isInstant;
    if (!isInstant) {
      addRequirements(hand);
    }
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!isInstant) {
      timer.restart();
    }
    hand.setWristGoal(wristGoal);
    BreakerLog.logEvent("SetHandWristGoal command instace started, (set goal: " + wristGoal.toString() + ")");
  }

  @Override
  public void execute() {
      if (!isInstant && timer.hasElapsed(IntakeConstants.WRIST_SET_GOAL_COMMAND_TIMEOUT_SEC)) {
        this.cancel();
      }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    if (interrupted) {
      BreakerLog.logEvent("SetHandWristGoal command instance FAILED, command timed out or was cancled");
    } else if (!isInstant) {
      BreakerLog.logEvent("SetHandWristGoal command instance SUCESSFULL, intake reached desired state");
    } else {
      BreakerLog.logEvent("SetHandWristGoal command ran and finished whout checks, desired end state not garrentieed");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hand.atWristGoal() || isInstant;
  }
}
