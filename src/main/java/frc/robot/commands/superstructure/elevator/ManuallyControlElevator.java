// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorControlMode;

public class ManuallyControlElevator extends CommandBase {
  /** Creates a new ManuallyControlElevator. */
  private Elevator elevator;
  private BreakerXboxController driverController;
  public ManuallyControlElevator(Elevator elevator, BreakerXboxController driverController) {
    this.elevator = elevator;
    this.driverController = driverController;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BreakerLog.getInstance().logSuperstructureEvent("ELEVATOR SWITCHED TO MANUAL CONTROL");
    elevator.setManual(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setManual(driverController.getLeftTrigger().get() - driverController.getRightTrigger().get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BreakerLog.getInstance().logSuperstructureEvent("ELEVATOR RETURNED TO AUTOMATIC CONTROL");
    elevator.setTarget(elevator.getHeight());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getCurrentControlMode() != ElevatorControlMode.MANUAL;
  }
}
