// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Elevator.ElevatorTargetState;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowElevatorIntakeAssembly extends SetSuperstructurePositionState {
  /** Creates a new StowElevatorIntakeAssembly. */
  public StowElevatorIntakeAssembly(Elevator elevator, Hand hand, boolean verifyIntakeAcutation) {
     super(elevator, hand, SuperstructurePositionState.STOW, verifyIntakeAcutation);
  }
}
