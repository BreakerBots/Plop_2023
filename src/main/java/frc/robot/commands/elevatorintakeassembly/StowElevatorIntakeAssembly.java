// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorintakeassembly;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorTargetState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.ActuatorMotorState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowElevatorIntakeAssembly extends SetElevatorIntakeAssemblyState {
  /** Creates a new StowElevatorIntakeAssembly. */
  public StowElevatorIntakeAssembly(Elevator elevator, Intake intake, boolean verifyIntakeAcutation) {
     super(elevator, intake, ElevatorTargetState.STOW, ActuatorMotorState.RETRACTING, verifyIntakeAcutation);
  }
}
