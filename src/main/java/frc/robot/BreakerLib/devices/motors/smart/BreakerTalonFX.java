// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors.smart;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.BreakerLib.devices.encoders.BreakerGenericEncoder;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Add your docs here. */
public class BreakerTalonFX extends TalonFX implements BreakerGenericSmartMotorController {
    private final VoltageOut voltageOut;
    private final DutyCycleOut dutyCycleOut;
    public BreakerTalonFX(PhoenixLicenceType licenceType, int deviceId, String canbus) {
        super(deviceId, canbus);
        voltageOut = new VoltageOut(0.0, licenceType.isProLicenced(), false);
        dutyCycleOut = new DutyCycleOut(0.0, licenceType.isProLicenced(), false);
    } 

    public BreakerTalonFX(PhoenixLicenceType licenceType, int deviceId) {
        this(licenceType, deviceId, "");
    }

    @Override
    public void runSelfTest() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public DeviceHealth getHealth() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public String getFaults() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public String getDeviceName() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public boolean hasFault() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void setDeviceName(String newName) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getDrawCurrent() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void setBrakeMode() {
        
    }

    @Override
    public BreakerGenericEncoder getSelectedEncoder() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public BreakerGenericEncoder getEncoder(int encoderNum) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void set(double speed) {
        setControl(dutyCycleOut.withOutput(speed));
    }

    @Override
    public void setVoltage(double volts) {
        setControl(voltageOut.withOutput(volts));
    }

    public static enum PhoenixLicenceType {
        STANDARD(false),
        PRO(true);
 
        private final boolean isPro;
        private PhoenixLicenceType(boolean isPro) {
            this.isPro = isPro;
        }

        public boolean isProLicenced() {
            return isPro;
        }
    }

    public static class TalonFXRotorEncoder implements BreakerGenericEncoder {

        @Override
        public void runSelfTest() {
            // TODO Auto-generated method stub
            
        }

        @Override
        public DeviceHealth getHealth() {
            // TODO Auto-generated method stub
            return null;
        }

        @Override
        public String getFaults() {
            // TODO Auto-generated method stub
            return null;
        }

        @Override
        public String getDeviceName() {
            // TODO Auto-generated method stub
            return null;
        }

        @Override
        public boolean hasFault() {
            // TODO Auto-generated method stub
            return false;
        }

        @Override
        public void setDeviceName(String newName) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public double getEncoderAbsolutePosition() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public double getEncoderRelativePosition() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public double getEncoderVelocity() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public double setEncoderPosition(double newPosition) {
            // TODO Auto-generated method stub
            return 0;
        }

    }
    

}
