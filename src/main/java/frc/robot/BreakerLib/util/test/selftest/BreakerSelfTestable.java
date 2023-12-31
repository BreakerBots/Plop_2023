// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.selftest;

/** Interface for all devises capable of automated self testing */
public interface BreakerSelfTestable {
    /** A method called pereodicly by the {@link SelfTest} class, 
     * sets device helth and fault string when called*/
    public abstract void runSelfTest();

    /** Should return the {@link DeviceHealth}.NOMINAL value unless a device fault or error is present */
    public abstract DeviceHealth getHealth();

    /** @return The String representation of this device's current faults */
    public abstract String getFaults();

    /** @return The name of this device, returns a default name if not manualy set */
    public abstract String getDeviceName();

    /** @return A boolean representing wether or not this devices has expierenced a fault */
    public abstract boolean hasFault();

    /** Sets the device's name as returned by the {@link BreakerSelfTestable#getDeviceName()} method */
    public abstract void setDeviceName(String newName);


    /** Infrestructure class for {@link BreakerSelfTestableBase} {@link BreakerGenericLoopedDevice} and simeler classes that directly inheret from {@link BreakerSelfTestable}.
     *  Exists to require a unified featureset between all {@link BreakerSelfTestable} child classes.
     * (NOTE: To be imlemented globaly following Subsystem/SubsystemBase rework in WPILib 2024)
     * 
     */
    public static class BreakerSelfTestableCore {
        public String faultStr, deviceName;
        public DeviceHealth health;
        public BreakerSelfTestableCore() {
            faultStr = "";
            deviceName = " Unknown_Device ";
            health = DeviceHealth.NOMINAL;
        }

        public boolean hasFault() {
            return health != DeviceHealth.NOMINAL;
        }

    }
}
