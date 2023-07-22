// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.motorgroups;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class BreakerNeoDiffDriveMotorGroup extends BreakerDiffDriveMotorGroup {

    public BreakerNeoDiffDriveMotorGroup(boolean invert, CANSparkMax... motors) {
        super(
        invert, 
        () -> {return motors[0].getEncoder().getPosition();}, 
        () -> {return motors[0].getEncoder().getVelocity() / 60.0;}, 
        motors
        );
    }



    

    


    
}
