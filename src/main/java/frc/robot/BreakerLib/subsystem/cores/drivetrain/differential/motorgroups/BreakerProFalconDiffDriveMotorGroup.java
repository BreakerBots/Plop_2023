// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.motorgroups;

import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class BreakerProFalconDiffDriveMotorGroup extends BreakerDiffDriveMotorGroup {

    public BreakerProFalconDiffDriveMotorGroup(boolean invert, TalonFX... motors) {
        super(
        invert, 
        () -> {return motors[0].getRotorPosition().getValue();}, 
        () -> {return motors[0].getRotorVelocity().getValue();}, 
        motors
        );
    }

    


    
}
