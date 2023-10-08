// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.ctre.BreakerFalconSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.ctre.BreakerProFalconSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.rev.BreakerNeoSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.BreakerGenericSwerveModuleDriveMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerProFalconSwerveModuleDriveMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerFalconSwerveModuleDriveMotor.TalonFXControlOutputUnits;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerProFalconSwerveModuleDriveMotor.ProTalonFXControlOutputUnits;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerFalconSwerveModuleDriveMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.rev.BreakerNeoSwerveModuleDriveMotor;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;

/** Add your docs here. */
public class BreakerSwerveModuleBuilder {
    private BreakerSwerveModuleConfig config;
    private BreakerGenericSwerveModuleAngleMotor angleMotor;
    private BreakerGenericSwerveModuleDriveMotor driveMotor;
    private BreakerSwerveModuleBuilder(BreakerSwerveModuleConfig config) {
        this.config = config;
    }

    public static BreakerSwerveModuleBuilder getInstance(BreakerSwerveModuleConfig config) {
        return new BreakerSwerveModuleBuilder(config);
    }

    /** Sets the module's angle motor to a CTRE TalonFX with STANDARD LICENCING
     * 
     * @param motor The NON PRO LICENCED TalonFX object to use
     * @param encoder The {@link BreakerSwerveAzimuthEncoder} to use for anguler positioning
     * @param encoderAbsoluteAngleOffsetRotations The offset of the positioning encoder's native reading such that 0rot is forward and the same for all modules 
     * @param isMotorInverted Polarity of motor output, used to match motor output to encoder reading
     * @return This {@link BreakerSwerveModuleBuilder} so that config calls can be chained
     */
    public BreakerSwerveModuleBuilder withFalconAngleMotor(TalonFX motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetRotations, boolean isMotorInverted) {
        angleMotor = new BreakerFalconSwerveModuleAngleMotor(motor, encoder, encoderAbsoluteAngleOffsetRotations, config.getAzimuthGearRatio(), config.getAngleSupplyCurrentLimit(), isMotorInverted, config.getAnglePIDConfig());
        return this;
    }

    /** Sets the module's drive motor to a CTRE TalonFX with STANDARD LICENCING
     * 
     * @param motor The NON PRO LICENCED TalonFX object to use
     * @param controlOutputUnits A {@link TalonFXControlOutputUnits} enum that sets the output and calculation units of the motor's velocity closed loop contro. PID and FF units MUST match slected units!
     * @param isMotorInverted Polarity of motor output, used to match motor output to encoder reading
     * @return This {@link BreakerSwerveModuleBuilder} so that config calls can be chained
     */
    public BreakerSwerveModuleBuilder withFalconDriveMotor(TalonFX motor, TalonFXControlOutputUnits controlOutputUnits, boolean isMotorInverted) {
        driveMotor = new BreakerFalconSwerveModuleDriveMotor(motor, config.getDriveGearRatio(), config.getWheelDiameter(), config.getDriveSupplyCurrentLimit(), isMotorInverted, config.getDriveArbFF(), config.getDrivePIDConfig(), controlOutputUnits);
        return this;
    }

    
    /** Sets the module's angle motor to a CTRE TalonFX with PRO LICENCING
     * 
     * @param motor The STANDARD LICENCED TalonFX object to use
     * @param encoder The {@link BreakerSwerveAzimuthEncoder} to use for anguler positioning
     * @param encoderAbsoluteAngleOffsetRotations The offset of the positioning encoder's native reading such that 0rot is forward and the same for all modules 
     * @param isMotorInverted Polarity of motor output, used to match motor output to encoder reading
     * @return This {@link BreakerSwerveModuleBuilder} so that config calls can be chained
     */
    public BreakerSwerveModuleBuilder withProFalconAngleMotor(TalonFX motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetRotations, boolean isMotorInverted) {
        angleMotor = new BreakerProFalconSwerveModuleAngleMotor(motor, encoder, config.getAzimuthGearRatio(), encoderAbsoluteAngleOffsetRotations, config.getAngleSupplyCurrentLimit(), isMotorInverted, config.getAnglePIDConfig());
        return this;
    }

    /** Sets the module's drive motor to a CTRE TalonFX with PRO LICENCING
     * 
     * @param motor The NON PRO LICENCED TalonFX object to use
     * @param controlOutputUnits A {@link ProTalonFXControlOutputUnits} enum that sets the output and calculation units of the motor's velocity closed loop contro. PID and FF units MUST match slected units!
     * @param isMotorInverted Polarity of motor output, used to match motor output to encoder reading
     * @return This {@link BreakerSwerveModuleBuilder} so that config calls can be chained
     */
    public BreakerSwerveModuleBuilder withProFalconDriveMotor(TalonFX motor, ProTalonFXControlOutputUnits controlOutputUnits, boolean isMotorInverted) {
        driveMotor = new BreakerProFalconSwerveModuleDriveMotor(motor, config.getDriveGearRatio(), config.getWheelDiameter(), config.getDriveSupplyCurrentLimit(), isMotorInverted, config.getDriveArbFF(), config.getDrivePIDConfig(), controlOutputUnits);
        return this;
    }

    
    /** Sets the module's angle motor to a REV Spark MAX powering a NEO BLDC motor
     * 
     * @param motor The CANSparkMax object to use
     * @param encoder The {@link BreakerSwerveAzimuthEncoder} to use for anguler positioning
     * @param encoderAbsoluteAngleOffsetRotations The offset of the positioning encoder's native reading such that 0rot is forward and the same for all modules 
     * @param isMotorInverted Polarity of motor output, used to match motor output to encoder reading
     * @return This {@link BreakerSwerveModuleBuilder} so that config calls can be chained
     */
    public BreakerSwerveModuleBuilder withNEOAngleMotor(CANSparkMax motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetRotations, boolean isMotorInverted) {
        angleMotor = new BreakerNeoSwerveModuleAngleMotor(motor, encoder, encoderAbsoluteAngleOffsetRotations, (int) config.getAngleSupplyCurrentLimit(), isMotorInverted, config.getAnglePIDConfig());
        return this;
    }

    
    /** Sets the module's drive motor to a REV Spark MAX powering a NEO BLDC motor
     * 
     * @param motor The CANSparkMax object to use
     * @param isMotorInverted Polarity of motor output, used to match motor output to encoder reading
     * @return This {@link BreakerSwerveModuleBuilder} so that config calls can be chained
     */
    public BreakerSwerveModuleBuilder withNEODriveMotor(CANSparkMax motor, boolean isMotorInverted) {
        driveMotor = new BreakerNeoSwerveModuleDriveMotor(motor, config.getDriveGearRatio(), config.getWheelDiameter(), (int) config.getDriveSupplyCurrentLimit(), isMotorInverted, config.getDriveArbFF(), config.getDrivePIDConfig());
        return this;
    }

    /** Creates
     * 
     * @param wheelPositionRelativeToRobot
     * @return
     */
    public BreakerSwerveModule createSwerveModule(Translation2d wheelPositionRelativeToRobot) {
        return new BreakerSwerveModule(driveMotor, angleMotor, wheelPositionRelativeToRobot);
    }

    public static class BreakerSwerveModuleConfig {
        private double driveGearRatio, azimuthGearRatio, wheelDiameter, angleSupplyCurrentLimit, driveSupplyCurrentLimit;
        private BreakerSwerveMotorPIDConfig anglePIDConfig, drivePIDConfig;
        private BreakerArbitraryFeedforwardProvider driveArbFF;
        public BreakerSwerveModuleConfig(double driveGearRatio, double azimuthGearRatio, double wheelDiameter, double angleSupplyCurrentLimit, double driveSupplyCurrentLimit, BreakerSwerveMotorPIDConfig anglePIDConfig, BreakerSwerveMotorPIDConfig drivePIDConfig, BreakerArbitraryFeedforwardProvider driveArbFF) {
            this.driveGearRatio = driveGearRatio;
            this.azimuthGearRatio = azimuthGearRatio;
            this.wheelDiameter = wheelDiameter;
            this.drivePIDConfig = drivePIDConfig;
            this.anglePIDConfig = anglePIDConfig;
            this.angleSupplyCurrentLimit = angleSupplyCurrentLimit;
            this.driveSupplyCurrentLimit = driveSupplyCurrentLimit;
            this.driveArbFF = driveArbFF;
        }

        public BreakerSwerveMotorPIDConfig getAnglePIDConfig() {
            return anglePIDConfig;
        }

        public BreakerSwerveMotorPIDConfig getDrivePIDConfig() {
            return drivePIDConfig;
        }

        public BreakerArbitraryFeedforwardProvider getDriveArbFF() {
            return driveArbFF;
        }

        public double getDriveGearRatio() {
            return driveGearRatio;
        }

        public double getAzimuthGearRatio() {
            return azimuthGearRatio;
        }

        public double getWheelDiameter() {
            return wheelDiameter;
        }

        public double getAngleSupplyCurrentLimit() {
            return angleSupplyCurrentLimit;
        }

        public double getDriveSupplyCurrentLimit() {
            return driveSupplyCurrentLimit;
        }
    }
}
