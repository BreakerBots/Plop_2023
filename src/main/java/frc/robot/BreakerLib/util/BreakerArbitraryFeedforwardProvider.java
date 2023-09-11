// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BreakerLib.util.math.interpolation.maps.BreakerGenericInterpolatingMap;

/**
 * A class that acts as a provider for arbitrary feedforward demand values used
 * with the Talon or SparkMax motor controller's integrated PID.
 */
public class BreakerArbitraryFeedforwardProvider {

    private BreakerGenericInterpolatingMap<Double, Double> ffMap;
    private double velocityCoefficient, staticFrictionCoefficient;
    private Function<Double, Double> ffFunc;
    private SimpleMotorFeedforward ffClass;

    private FeedForwardType ffType;
    private FeedForwardUnits calcUnits;

    /** Type of arbitrary feedforward provider. */
    private enum FeedForwardType {
        MAP_SUP,
        COEFFICIENTS,
        FUNCTION,
        FF_CLASS
    }

    public static enum FeedForwardUnits {
        VOLTAGE,
        DUTY_CYCLE,
        /** Do not apply any conversion or compensation to value, regardless of requested or calculated units */
        OTHER
    }

    /**
     * Creates a map-based ArbitraryFeedForwardProvider.
     * 
     * @param speedToFeedforwardValMap
     * @param calculationUnits Units returned by the FF calcualtion used (voltage or duty cycle)
     */
    public BreakerArbitraryFeedforwardProvider(
            BreakerGenericInterpolatingMap<Double, Double> speedToFeedforwardValMap, FeedForwardUnits calculationUnits) {
        ffMap = speedToFeedforwardValMap;
        ffType = FeedForwardType.MAP_SUP;
        calcUnits = calculationUnits;
    }

    /**
     * Creates a coefficient-based ArbitraryFeedForwardProvider.
     * 
     * @param staticFrictionCoefficient Feedforward kS coefficient.
     * @param velocityCoefficient Feedforward kV coefficient.
     * @param calculationUnits Units returned by the FF calcualtion used (voltage or duty cycle)
     */
    public BreakerArbitraryFeedforwardProvider(double staticFrictionCoefficient, double velocityCoefficient, FeedForwardUnits calculationUnits) {
        this.velocityCoefficient = velocityCoefficient;
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        ffType = FeedForwardType.COEFFICIENTS;
        calcUnits = calculationUnits;
    }

    /**
     * Creates a function-based ArbitraryFeedForwardProvider that takes input.
     * 
     * @param ffFunc Function that applies feedforward calculations to given
     *               velocity (m/s).
     * @param calculationUnits Units returned by the FF calcualtion used (voltage or duty cycle)
     */
    public BreakerArbitraryFeedforwardProvider(Function<Double, Double> ffFunc, FeedForwardUnits calculationUnits) {
        this.ffFunc = ffFunc;
        ffType = FeedForwardType.FUNCTION;
        calcUnits = calculationUnits;
    }

    /**
     * Creates a function-based ArbitraryFeedForwardProvider that takes no input.
     * 
     * @param ffSupplier Supplier function that provides feedforward results.
     * @param calculationUnits Units returned by the FF calcualtion used (voltage or duty cycle)
     */
    public BreakerArbitraryFeedforwardProvider(DoubleSupplier ffSupplier, FeedForwardUnits calculationUnits) {
        ffFunc = (Double x) -> (ffSupplier.getAsDouble());
        ffType = FeedForwardType.FUNCTION;
        calcUnits = calculationUnits;
    }

    /**
     * Creates an ArbitraryFeedForwardProvider with a standard feedforward object.
     * 
     * @param ffClass Feedforward object.
     * @param calculationUnits Units returned by the FF calcualtion used (voltage or duty cycle)
     */
    public BreakerArbitraryFeedforwardProvider(SimpleMotorFeedforward ffClass, FeedForwardUnits calculationUnits) {
        this.ffClass = ffClass;
        ffType = FeedForwardType.FF_CLASS;
        calcUnits = calculationUnits;
    }

    /** @return Percent output to be added to the desired motors's output to achieve the desired speed. */
    public double getArbitraryFeedforwardValue(double curSpeed, FeedForwardUnits returnUnits) {
        double feedForward = 0.0;
        if (curSpeed != 0.0) {
            switch (ffType) {
                case COEFFICIENTS:
                    feedForward = ((velocityCoefficient * curSpeed + (staticFrictionCoefficient * Math.signum(curSpeed))));
                    break;
                case MAP_SUP:
                    feedForward = ffMap.getInterpolatedValue(curSpeed);
                    break;
                case FUNCTION:
                    feedForward = ffFunc.apply(curSpeed);
                    break;
                case FF_CLASS:
                    feedForward = ffClass.calculate(curSpeed);
            }
        }
        if (calcUnits == FeedForwardUnits.DUTY_CYCLE && returnUnits == FeedForwardUnits.VOLTAGE) {
            return feedForward * RobotController.getBatteryVoltage();
        } else if (calcUnits == FeedForwardUnits.VOLTAGE && returnUnits == FeedForwardUnits.DUTY_CYCLE) {
            return feedForward / RobotController.getBatteryVoltage();
        }
        return feedForward;
    }


}
