// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors.smart;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLoggable;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestable;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Add your docs here. */
public class BreakerTalonFX extends TalonFX implements BreakerSelfTestable, BreakerLoggable{
    private PhoenixLicenceType licenceType;
    private DutyCycleOut dutyCycleRequest;
    private VoltageOut voltageRequest;
    public BreakerTalonFX(int deviceNumber, String canbus, PhoenixLicenceType licenceType) {
        super(deviceNumber, canbus);
        this.licenceType = licenceType;
        dutyCycleRequest = new DutyCycleOut(0.0, licenceType.isProLicenced(), false);
        voltageRequest = new VoltageOut(0.0, licenceType.isProLicenced(), false);
    }

    public BreakerTalonFX(int deviceNumber, PhoenixLicenceType licenceType) {
        this(deviceNumber, "rio", licenceType);
    }

    @Override
    public void set(double speed) {
        
    }

    @Override
    public void setVoltage(double volts) {
        
    }


    public enum PhoenixLicenceType {
        STANDARD,
        PRO;

        public boolean isProLicenced() {
            return this == PRO;
        }
    }

    @Override
    public void toLog(LogTable table) {
        // TODO Auto-generated method stub
        
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

    
}
