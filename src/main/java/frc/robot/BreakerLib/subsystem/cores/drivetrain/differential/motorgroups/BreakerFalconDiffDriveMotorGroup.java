// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.motorgroups;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;

/** Add your docs here. */
public class BreakerFalconDiffDriveMotorGroup extends BreakerDiffDriveMotorGroup {
    private TalonFX[] motors;
    public BreakerFalconDiffDriveMotorGroup(boolean invert, TalonFX... motors) {
        super(
        invert, 
        () -> {return motors[0].getRotorPosition().getValue();}, 
        () -> {return motors[0].getRotorVelocity().getValue();}, 
        NonFOCMotorControllerFalconWrapper.convertAll(motors)
        );
        this.motors = motors;
    }

    @Override
    public void setBrakeMode(boolean isEnabled) {
        BreakerPhoenix6Util.setBrakeMode(isEnabled, motors);
    }

    @Override
    public void setRotorPosition(double newPosition) {
        motors[0].setRotorPosition(newPosition);
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        StringBuilder work = new StringBuilder();
        for (TalonFX motor : motors) {
            Pair<DeviceHealth, String> motorFaultData = BreakerPhoenix6Util.checkMotorFaultsAndConnection(motor);
            if (motorFaultData.getFirst() != DeviceHealth.NOMINAL) {
                health = DeviceHealth.FAULT;
                work.append(" MOTOR ID (" + motor.getDeviceID() + ") FAULTS: " + motorFaultData.getSecond());
            }
        }

    }

    private static class NonFOCMotorControllerFalconWrapper implements MotorController {
        private TalonFX motor;
        private final DutyCycleOut setterControl = new DutyCycleOut(0.0, false, false);
  
        public NonFOCMotorControllerFalconWrapper(TalonFX motor) {
          this.motor = motor;
        }
  
        @Override
        public void set(double speed) {
          motor.setControl(setterControl.withOutput(speed));
        }
  
        @Override
        public double get() {
          return motor.get();
        }
  
        @Override
        public void setInverted(boolean isInverted) {
          motor.setInverted(isInverted);
        }
  
        @Override
        public boolean getInverted() {
          return motor.getInverted();
        }
  
        @Override
        public void disable() {
          motor.disable();
        }
  
        @Override
        public void stopMotor() {
          motor.stopMotor();
        }
  
        public static NonFOCMotorControllerFalconWrapper[] convertAll(TalonFX... motors) {
          NonFOCMotorControllerFalconWrapper[] wrapperInstances = new NonFOCMotorControllerFalconWrapper[motors.length];
          for (int i = 0; i < motors.length; i++) {
            wrapperInstances[i] = new NonFOCMotorControllerFalconWrapper(motors[i]);
          }
          return wrapperInstances;
        }
        
    }



    
}
