package frc.robot.BreakerLib.devices.sensors.imu.ctre;

import com.ctre.phoenix.sensors.PigeonIMU_Faults;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.devices.sensors.BreakerGenericMagnetometer;
import frc.robot.BreakerLib.devices.sensors.imu.BreakerGenericIMU;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

public class BreakerPigeon extends BreakerGenericIMU implements BreakerGenericMagnetometer {

    private WPI_PigeonIMU pigeon;

    /** Creates a new PigeonIMU object. */
    public BreakerPigeon(int deviceID) {
        pigeon = new WPI_PigeonIMU(deviceID);
        deviceName = "Pigeon_IMU (" + deviceID + ") ";
    }

    @Override
    public double getPitch() {
        return BreakerMath.angleModulus(pigeon.getPitch());
    }

    @Override
    public double getYaw() {
        return BreakerMath.angleModulus(pigeon.getYaw());
    }

    @Override
    public double getRoll() {
        return BreakerMath.angleModulus(pigeon.getRoll());
    }

    @Override
    public Rotation2d getPitchRotation2d() {
        return Rotation2d.fromDegrees(getPitch());
    }

    @Override
    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    @Override
    public Rotation2d getRollRotation2d() {
        return Rotation2d.fromDegrees(getRoll());
    }

    @Override
    public Rotation3d getRotation3d() {
        return new Rotation3d(getQuaternion());
    }

    @Override
    public double[] getRawAngles() {
        double[] RawYPR = new double[3];
        pigeon.getYawPitchRoll(RawYPR);
        return RawYPR;
    }

    @Override
    public double getRawPitch() {
        return getRawAngles()[1];
    }

    @Override
    public double getRawRoll() {
        return getRawAngles()[2];
    }

    @Override
    public double getRawYaw() {
        return getRawAngles()[0];
    }

    /** Does nothing. */
    @Override
    public void setPitch(double value) {
        // TODO Auto-generated method stub
    }

    @Override
    public void setYaw(double value) {
        pigeon.setYaw(0);
    }

    /** Does nothing. */
    @Override
    public void setRoll(double value) {

    }

    /** Sets yaw to 0 */
    @Override
    public void reset() {
        pigeon.setYaw(0);
    }

    public double[] getRawGyroRates() {
        double[] rawRates = new double[3];
        pigeon.getRawGyro(rawRates);
        return rawRates;
    }

    @Override
    public double getRawPitchRate() {
        return getRawGyroRates()[0];
    }

    @Override
    public double getRawRollRate() {
        return getRawGyroRates()[2];
    }

    @Override
    public double getRawYawRate() {
        return getRawGyroRates()[1];
    }

    @Override
    public double getPitchRate() {
        return getRawPitchRate();
    }

    @Override
    public double getYawRate() {
        return getRawYawRate();
    }

    @Override
    public double getRollRate() {
        return getRawRollRate();
    }

    /**
     * @return Biased accelerometer values. The raw accelerometer vals of
     *         the Pigeon are not normally accessible.
     *         <p>
     *         x = 0, y = 1, z = 2.
     */
    @Override
    public double[] getRawAccelerometerVals() {
        double[] newVals = new double[3];

        for (int i = 0; i < 3; i++) {
            newVals[i] = (BreakerMath.fixedToFloat(getBiasedAccelerometerValsShort()[i], 14) * 0.000508);
        }
        return newVals;
    }

    /**
     * @return Biased accelerometer values in native short format.
     *         <p>
     *         x = 0, y = 1, z = 2.
     */
    public short[] getBiasedAccelerometerValsShort() {
        short[] accelVals = new short[3];
        pigeon.getBiasedAccelerometer(accelVals);
        return accelVals;
    }

    /** @return Biased accelerometer x-value in m/s^2. */
    @Override
    public double getRawAccelX() {
        return (BreakerMath.fixedToFloat(getBiasedAccelerometerValsShort()[0], 14) * 0.000508);
    }

    /** @return Biased accelerometer y-value in m/s^2. */
    @Override
    public double getRawAccelY() {
        return (BreakerMath.fixedToFloat(getBiasedAccelerometerValsShort()[1], 14) * 0.000508);
    }

    @Override
    /** @return Biased accelerometer z-value in m/s^2. */
    public double getRawAccelZ() {
        return (BreakerMath.fixedToFloat(getBiasedAccelerometerValsShort()[2], 14) * 0.000508);
    }

    /**
     * @return Amount of time in seconds the Pigeon has been running. Maximum value
     *         of 255 secs.
     */
    public int getPigeonUpTime() {
        return pigeon.getUpTime();
    }

    @Override
    public Rotation3d getRawRotation3d() {
        return new Rotation3d(Math.toRadians(getRawAngles()[2]), Math.toRadians(getRawAngles()[1]),
                Math.toRadians(getRawAngles()[0]));
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        PigeonIMU_Faults curFaults = new PigeonIMU_Faults();
        pigeon.getFaults(curFaults);
        if (curFaults.hasAnyFault()) {
            health = DeviceHealth.INOPERABLE;
            faultStr += " unknown_fault ";
        }
        if (pigeon.getFirmwareVersion() == -1) {
            health = DeviceHealth.INOPERABLE;
            faultStr += " device_disconnected ";
        }
    }

    @Override
    public double[] getRawFieldStrengths() {
        short[] rawShorts = new short[] { 3 };
        pigeon.getRawMagnetometer(rawShorts);
        return new double[] { (double) rawShorts[0] * 0.6, (double) rawShorts[1] * 0.6, (double) rawShorts[2] * 0.6 };
    }

    @Override
    public double[] getBiasedFieldStrengths() {
        short[] rawShorts = new short[] { 3 };
        pigeon.getBiasedMagnetometer(rawShorts);
        return new double[] { (double) rawShorts[0] * 0.6, (double) rawShorts[1] * 0.6, (double) rawShorts[2] * 0.6 };
    }

    @Override
    public double getCompassFieldStrength() {
        return pigeon.getCompassFieldStrength();
    }

    @Override
    public double getCompassHeading() {
        return MathUtil.angleModulus(pigeon.getCompassHeading());
    }

    @Override
    public double getRawCompassHeading() {
        return pigeon.getCompassHeading();
    }

    @Override
    public Quaternion getQuaternion() {
        double[] quat = new double[4];
        pigeon.get6dQuaternion(quat);
        return new Quaternion(quat[0], quat[1], quat[2], quat[3]);
    }

    /**
     * Does nothing. Calibration is done on boot and can be performed with Phoenix
     * Tuner.
     */
    public void calibrate() {
        pigeon.calibrate();
    }

    @Override
    public void close() throws Exception {
        pigeon.close();
        
    }
}
