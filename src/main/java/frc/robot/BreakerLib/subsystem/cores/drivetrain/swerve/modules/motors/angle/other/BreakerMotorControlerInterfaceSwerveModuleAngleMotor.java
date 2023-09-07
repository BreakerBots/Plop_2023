// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.other;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerSwerveAzimuthControler;

/** Add your docs here. */
public class BreakerMotorControlerInterfaceSwerveModuleAngleMotor extends BreakerGenericSwerveModuleAngleMotor {
    private BreakerSwerveAzimuthControler controler;
    private BreakerSwerveAzimuthEncoder encoder;
    private Rotation2d tgtAng;
    private MotorController motor;
    public BreakerMotorControlerInterfaceSwerveModuleAngleMotor(MotorController motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetDegrees, boolean isMotorInverted,  BreakerSwerveMotorPIDConfig pidConfig) {
        motor.setInverted(isMotorInverted);
        controler = new BreakerSwerveAzimuthControler(motor, encoder, pidConfig);
        this.encoder = encoder;
        tgtAng = new Rotation2d();
        this.motor = motor;
    }

    @Override
    public void runSelfTest() {}

    @Override
    public void setTargetAngle(Rotation2d targetAngle) {
        controler.setTargetAngle(targetAngle);
    }
 
    @Override
    public double getAbsoluteAngle() {
        return encoder.getAbsolute();
    }

    @Override
    public double getRelativeAngle() {
        return encoder.getRelative();
    }

    @Override
    public void setBrakeMode(boolean isEnabled) {}

    @Override
    public Rotation2d getTargetAngle() {
        return tgtAng;
    }

    @Override
    public double getSupplyCurrent() {
        return 0;
    }

    @Override
    public double getMotorOutput() {
        return motor.get();
    }
}
