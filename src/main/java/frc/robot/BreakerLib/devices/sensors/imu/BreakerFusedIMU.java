// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.imu;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/**
 * Fuses multiple IMUs together through a weighted average to produce steadier
 * results.
 */
public class BreakerFusedIMU extends BreakerGenericIMU {

    private BreakerGenericIMU[] imuArr;
    private double[] weightArr, dataArr;

    /**
     * Creates a fused IMU with 2 or more IMUs and corresponding weights. Make sure
     * to call removeFaultyIMUData() periodically to avoid data from faulty IMUs
     * polluting the average.
     * 
     * @param imuWeightPairs IMU and assosciated weight. Weights do not need to sum
     *                       to 1.
     */
    public BreakerFusedIMU(Pair<BreakerGenericIMU, Double>... imuWeightPairs) {
        int pairArrLen = imuWeightPairs.length;
        dataArr = new double[pairArrLen];

        for (int i = 0; i < pairArrLen; i++) {
            imuArr[i] = imuWeightPairs[i].getFirst();
            weightArr[i] = imuWeightPairs[i].getSecond();
        }
    }

    /**
     * @return IMUs fused together. Use if you want to access only one IMU at a
     *         time.
     */
    public BreakerGenericIMU[] getIMUs() {
        return imuArr;
    }

    /** Sets weight of faulty IMUs to 0. Should be called periodically. */
    public void removeFaultyIMUData() {
        for (int i = 0; i < imuArr.length; i++) {
            if (imuArr[i].getHealth() != DeviceHealth.NOMINAL) {
                weightArr[i] = 0;
            }
        }
    }

    @Override
    public double getPitch() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getPitch();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    @Override
    public double getRoll() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getRoll();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    @Override
    public Rotation2d getPitchRotation2d() {
        return Rotation2d.fromDegrees(getPitch());
    }

    @Override
    public Rotation2d getRollRotation2d() {
        return Rotation2d.fromDegrees(getRoll());

    }

    @Override
    public Quaternion getQuaternion() {
        return getRotation3d().getQuaternion();
    }

    @Override
    public Rotation3d getRotation3d() {
        return new Rotation3d(getRoll(), getPitch(), getYaw());
    }

    @Override
    public double[] getRawAngles() {
        return new double[] { getRawYaw(), getRawPitch(), getRawRoll() };
    }

    @Override
    public double getRawPitch() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getRawPitch();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    @Override
    public double getRawRoll() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getRawRoll();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    public void setPitch(double value) {
        for (BreakerGenericIMU imu : imuArr) {
            imu.setPitch(value);
        }
    }

    public void setRoll(double value) {
        for (BreakerGenericIMU imu : imuArr) {
            imu.setRoll(value);
        }
    }

    @Override
    public double[] getRawGyroRates() {
        return new double[] { getRawYawRate(), getRawPitchRate(), getRawRollRate() };
    }

    @Override
    public double getRawPitchRate() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getRawPitchRate();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    @Override
    public double getRawRollRate() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getRawRollRate();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    @Override
    public double getPitchRate() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getPitchRate();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    @Override
    public double getRollRate() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getRollRate();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    @Override
    public Rotation3d getRawRotation3d() {
        return new Rotation3d(getRawRoll(), getRawPitch(), getRawYaw());
    }

    public void reset() {
        for (BreakerGenericIMU imu : imuArr) {
            imu.reset();
        }
    }

    @Override
    public double getYaw() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getYaw();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    @Override
    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public void setYaw(double value) {
        for (BreakerGenericIMU imu : imuArr) {
            imu.setYaw(value);
        }
    }

    @Override
    public double getYawRate() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getYawRate();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    public void calibrate() {
        for (BreakerGenericIMU imu : imuArr) {
            imu.calibrate();
        }
    }

    @Override
    public double getRawYaw() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getRawYaw();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    @Override
    public double getRawYawRate() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getRawYawRate();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    @Override
    public double[] getRawAccelerometerVals() {
        return new double[] { getRawAccelX(), getRawAccelY(), getRawAccelZ() };
    }

    @Override
    public double getRawAccelX() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getRawAccelX();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    @Override
    public double getRawAccelY() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getRawAccelY();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    @Override
    public double getRawAccelZ() {
        for (int i = 0; i <= imuArr.length; i++) {
            dataArr[i] = imuArr[i].getRawAccelZ();
        }
        return BreakerMath.getWeightedAvg(dataArr, weightArr);
    }

    public void runSelfTest() {
        for (BreakerGenericIMU imu : imuArr) {
            imu.runSelfTest();
        }
    }

    @Override
    public void close() throws Exception {
        for (BreakerGenericIMU imu: imuArr) {
            imu.close();
        }
    } 
}
