// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public enum GamePieceType2 {
    CUBE,
    CONE;

    public boolean isCone() {
        return !isCube();
    }

    public boolean isCube() {
        return this.ordinal() == 0;
    }


}
