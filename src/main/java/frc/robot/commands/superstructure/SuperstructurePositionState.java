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
    PLACE_CUBE_MID(ElevatorTargetState.PLACE_CUBE_MID, WristGoal.PLACE_CUBE_MID),
    PLACE_CUBE_HIGH(ElevatorTargetState.PLACE_CUBE_HIGH, WristGoal.PLACE_CUBE_HIGH),
    PICKUP_GROUND_CONE(ElevatorTargetState.PICKUP_GROUND_CONE, WristGoal.PICKUP_GROUND_CONE),
    PICKUP_GROUND_CUBE(ElevatorTargetState.PICKUP_GROUND_CUBE, WristGoal.PICKUP_GROUND_CUBE),
    PICKUP_SINGLE_SUBSTATION_CONE(ElevatorTargetState.PICKUP_SINGLE_SUBSTATION_CONE, WristGoal.PICKUP_SINGLE_SUBSTATION_CONE),
    PICKUP_SINGLE_SUBSTATION_CUBE(ElevatorTargetState.PICKUP_SINGLE_SUBSTATION_CUBE, WristGoal.PICKUP_SINGLE_SUBSTATION_CUBE),
    PICKUP_DOUBLE_SUBSTATION_CONE(ElevatorTargetState.PICKUP_DOUBLE_SUBSTATION_CONE, WristGoal.PICKUP_DOUBLE_SUBSTATION_CONE),
    PICKUP_DOUBLE_SUBSTATION_CUBE(ElevatorTargetState.PICKUP_DOUBLE_SUBSTATION_CUBE, WristGoal.PICKUP_DOUBLE_SUBSTATION_CUBE),
    STOW(ElevatorTargetState.STOW, WristGoal.STOW);

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
