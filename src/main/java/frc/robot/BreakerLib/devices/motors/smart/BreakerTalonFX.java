// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors.smart;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.devices.encoders.BreakerGenericEncoder;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;

/** Add your docs here. */
public class BreakerTalonFX extends TalonFX implements BreakerGenericSmartMotorController {
    private final VoltageOut voltageOut;
    private final DutyCycleOut dutyCycleOut;
    private final BreakerGenericEncoder rotorEncoder, selectedEncoder;
    private String faultStr, deviceName;
    private DeviceHealth health;
    public BreakerTalonFX(PhoenixLicenceType licenceType, int deviceId, String canbus) {
        super(deviceId, canbus);
        voltageOut = new VoltageOut(0.0, licenceType.isProLicenced(), false);
        dutyCycleOut = new DutyCycleOut(0.0, licenceType.isProLicenced(), false);
        rotorEncoder = new TalonFXRotorEncoder(this);
        selectedEncoder = new TalonFXSelectedEncoder(this);
        health = DeviceHealth.NOMINAL;
        faultStr = "";
        deviceName = String.format("BreakerTalonFX_(ID: %d. BUS: %s)", deviceId, canbus);
    } 

    public BreakerTalonFX(PhoenixLicenceType licenceType, int deviceId) {
        this(licenceType, deviceId, "");
    }

    @Override
    public void runSelfTest() {
        health = DeviceHealth.NOMINAL;
        faultStr = "";
        Pair<DeviceHealth, String> faultPair = BreakerPhoenix6Util.checkMotorFaultsAndConnection(this);
        if (faultPair.getFirst() != DeviceHealth.NOMINAL) {
            health = faultPair.getFirst();
            faultStr = faultPair.getSecond();
        }
    }

    @Override
    public DeviceHealth getHealth() {
        return health;
    }

    @Override
    public String getFaults() {
        return faultStr;
    }

    @Override
    public String getDeviceName() {
        return deviceName;
    }

    @Override
    public boolean hasFault() {
        return health != DeviceHealth.NOMINAL;
    }

    @Override
    public void setDeviceName(String newName) {
       deviceName = newName;
    }

    @Override
    public double getDrawCurrent() {
        return getSupplyCurrent().getValue();
    }

    @Override
    public void setBrakeMode(boolean isEnabled) {
        BreakerPhoenix6Util.setBrakeMode(this, isEnabled);
    }

    @Override
    public BreakerGenericEncoder getSelectedEncoder() {
        return new TalonFXSelectedEncoder(this);
    }

    @Override
    public BreakerGenericEncoder getRotorEncoder() {
        return new TalonFXRotorEncoder(this);
    }


    @Override
    public BreakerGenericEncoder getEncoder(int encoderNum) {
        switch(encoderNum) {
            case 0:
                return getRotorEncoder();
            case 1:
            default:
                return getSelectedEncoder();
        }
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
        private String deviceName;
        private TalonFX talonFX;
        public TalonFXRotorEncoder(TalonFX talonFX) {
            deviceName = "TalonFXRotorEncoder";
            this.talonFX = talonFX;
        }

        @Override
        public void runSelfTest() {}

        @Override
        public DeviceHealth getHealth() {
            return DeviceHealth.NOMINAL;
        }

        @Override
        public String getFaults() {
            return "";
        }

        @Override
        public String getDeviceName() {
            return deviceName;
        }

        @Override
        public boolean hasFault() {
            return false;
        }

        @Override
        public void setDeviceName(String newName) {
           deviceName = newName;
        }

        @Override
        public double getEncoderAbsolutePosition() {
            return MathUtil.inputModulus(getEncoderRelativePosition(), -0.5, 0.5);
        }

        @Override
        public double getEncoderRelativePosition() {
            return talonFX.getRotorPosition().getValue();
        }

        @Override
        public double getEncoderVelocity() {
            return talonFX.getRotorVelocity().getValue();
        }

        @Override
        public void setEncoderPosition(double newPosition) {
            talonFX.setRotorPosition(newPosition);
        }

    }

    public static class TalonFXSelectedEncoder implements BreakerGenericEncoder {
        private String deviceName;
        private TalonFX talonFX;
        public TalonFXSelectedEncoder(TalonFX talonFX) {
            deviceName = "TalonFXSelectedEncoder";
            this.talonFX = talonFX;
        }

        @Override
        public void runSelfTest() {}

        @Override
        public DeviceHealth getHealth() {
            return DeviceHealth.NOMINAL;
        }

        @Override
        public String getFaults() {
            return "";
        }

        @Override
        public String getDeviceName() {
            return deviceName;
        }

        @Override
        public boolean hasFault() {
            return false;
        }

        @Override
        public void setDeviceName(String newName) {
           deviceName = newName;
        }

        @Override
        public double getEncoderAbsolutePosition() {
            return MathUtil.inputModulus(getEncoderRelativePosition(), -0.5, 0.5);
        }

        @Override
        public double getEncoderRelativePosition() {
            return talonFX.getPosition().getValue();
        }

        @Override
        public double getEncoderVelocity() {
            return talonFX.getVelocity().getValue();
        }

        @Override
        public void setEncoderPosition(double newPosition) {
            talonFX.setRotorPosition(newPosition);
        }

    }
}
