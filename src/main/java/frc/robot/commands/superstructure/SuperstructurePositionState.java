// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import frc.robot.subsystems.Elevator.ElevatorTargetState;
import frc.robot.subsystems.Hand.WristGoal;

/** Add your docs here. */
public enum SuperstructurePositionState {
    PLACE_HYBRID(ElevatorTargetState.PLACE_HYBRID, WristGoal.PLACE_HYBRID),
    PLACE_CONE_MID(ElevatorTargetState.PLACE_CONE_MID, WristGoal.PLACE_CONE_MID),
    PLACE_CONE_HIGH(ElevatorTargetState.PLACE_CONE_HIGH, WristGoal.PLACE_CONE_HIGH),
    PLACE_CUBE_MID(WristGoalType.PLACE, Rotation2d.fromDegrees(0)),
    PLACE_CUBE_HIGH(WristGoalType.PLACE, Rotation2d.fromDegrees(0)),
    PICKUP_GROUND_CONE(WristGoalType.PICKUP, Rotation2d.fromDegrees(0)),
    PICKUP_GROUND_CUBE(WristGoalType.PICKUP, Rotation2d.fromDegrees(0)),
    PICKUP_SINGLE_SUBSTATION_CONE(WristGoalType.PICKUP, Rotation2d.fromDegrees(0)),
    PICKUP_SINGLE_SUBSTATION_CUBE(WristGoalType.PICKUP, Rotation2d.fromDegrees(0)),
    PICKUP_DOUBLE_SUBSTATION_CONE(WristGoalType.PICKUP, Rotation2d.fromDegrees(0)),
    PICKUP_DOUBLE_SUBSTATION_CUBE(WristGoalType.PICKUP, Rotation2d.fromDegrees(0)),
    STOW(WristGoalType.STOW, Rotation2d.fromDegrees(0));

    private final ElevatorTargetState elevatorTargetState;
    private final WristGoal wristGoal;
    private SuperstructurePositionState(ElevatorTargetState elevatorTargetState, WristGoal wristGoal) {
        this.elevatorTargetState = elevatorTargetState;
        this.wristGoal = wristGoal;
    }

    public ElevatorTargetState getElevatorTargetState() {
        return elevatorTargetState;
    }

    public WristGoal getWristGoal() {
        return wristGoal;
    }
}
