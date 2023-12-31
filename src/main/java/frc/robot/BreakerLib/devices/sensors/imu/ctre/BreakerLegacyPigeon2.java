// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.BreakerLib.devices.sensors.imu.ctre;

import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.devices.sensors.BreakerGenericMagnetometer;
import frc.robot.BreakerLib.devices.sensors.imu.BreakerGenericIMU;
import frc.robot.BreakerLib.physics.vector.BreakerVector3;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/* CTRE Pigeon 2 implementing the Breaker device interface and Breaker IMU interface,  */
@Deprecated
public class BreakerLegacyPigeon2 extends BreakerGenericIMU implements BreakerGenericMagnetometer {
  private WPI_Pigeon2 pigeon;

  /** Creates a new PigeonIMU 2 object. */
  public BreakerLegacyPigeon2(int deviceID) {
    pigeon = new WPI_Pigeon2(deviceID);
    deviceName = "Pigeon2_IMU (" + deviceID + ") ";
  }

  /** Creates a new PigeonIMU 2 object. */
  public BreakerLegacyPigeon2(int deviceID, String busName) {
    pigeon = new WPI_Pigeon2(deviceID, busName);
    deviceName = "Pigeon2_IMU (" + deviceID + ") ";
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
  public double getRawYaw() {
    return getRawAngles()[0];
  }

  @Override
  public double getRawPitch() {
    return getRawAngles()[1];
  }

  @Override
  public double getRawRoll() {
    return getRawAngles()[2];
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
    return getRawGyroRates()[1];
  }

  @Override
  public double getRawYawRate() {
    return getRawGyroRates()[2];
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

  @Override
  public double[] getRawAccelerometerVals() {
    double[] newVals = new double[3];
    for (int i = 0; i < 3; i++) {
      newVals[i] = (BreakerMath.fixedToFloat(getRawAccelerometerValsShort()[i], 14) * 0.000508);
    }
    return newVals;
  }

  public short[] getRawAccelerometerValsShort() {
    short[] accelVals = new short[3];
    pigeon.getBiasedAccelerometer(accelVals);
    return accelVals;
  }

  @Override
  /** @return Unbiased accelerometer x-value in G */
  public double getRawAccelX() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerValsShort()[0], 14) * 0.000508);
  }

  @Override
  /** @return Unbiased accelerometer y-value in G. */
  public double getRawAccelY() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerValsShort()[1], 14) * 0.000508);
  }

  @Override
  /** @return Unbiased accelerometer z-value in G. */
  public double getRawAccelZ() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerValsShort()[2], 14) * 0.000508);
  }

  /** @return Pigeon's runtime in seconds (max of 255) */
  public int getPigeonUpTime() {

    return pigeon.getUpTime();
  }

  public BreakerVector3 getGravityVector() {
    double[] gravVec = new double[3];
    pigeon.getGravityVector(gravVec);
    return new BreakerVector3(gravVec[0], gravVec[1], gravVec[2]);
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
    Pigeon2_Faults curFaults = new Pigeon2_Faults();
    pigeon.getFaults(curFaults);

    if (curFaults.HardwareFault) {
      health = DeviceHealth.INOPERABLE;
      faultStr += " hardware_fault ";
    }
    if (curFaults.MagnetometerFault) {
      health = DeviceHealth.INOPERABLE;
      faultStr += " mag_fault ";
    }
    if (curFaults.GyroFault) {
      health = DeviceHealth.INOPERABLE;
      faultStr += "  gyro_fault ";
    }
    if (curFaults.AccelFault) {
      health = DeviceHealth.INOPERABLE;
      faultStr += " accel_fault ";
    }
    if (curFaults.UnderVoltage) {
      health = (health != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : health;
      faultStr += " under_6.5V ";
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
