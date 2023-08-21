// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

/** Add your docs here. */
public class DynamicSlewRateLimiter extends SlewRateLimiter {
    private final DoubleSupplier positiveRateLimitSupplier;
    private final  DoubleSupplier negativeRateLimitSupplier;
    private double prevVal;
    private double prevTime;

    public DynamicSlewRateLimiter(DoubleSupplier rateLimitSupplier) {
        this(rateLimitSupplier, 0.0);
    }


    public DynamicSlewRateLimiter(DoubleSupplier rateLimitSupplier, double initialValue) {
        this(rateLimitSupplier, () -> -rateLimitSupplier.getAsDouble(), initialValue);
    }

    public DynamicSlewRateLimiter(DoubleSupplier positiveRateLimitSupplier, DoubleSupplier negativeRateLimitSupplier, double initalValue) {
        super(positiveRateLimitSupplier.getAsDouble(), negativeRateLimitSupplier.getAsDouble(), initalValue);
        this.negativeRateLimitSupplier = negativeRateLimitSupplier;
        this.positiveRateLimitSupplier = positiveRateLimitSupplier;
        this.prevVal = initalValue;
        prevTime = MathSharedStore.getTimestamp();
    }

    @Override
    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - prevTime;
        prevVal +=
            MathUtil.clamp(
                input - prevVal,
                negativeRateLimitSupplier.getAsDouble() * elapsedTime,
                positiveRateLimitSupplier.getAsDouble() * elapsedTime);
        prevTime = currentTime;
        return prevVal;
    }

    @Override
    public void reset(double value) {
        // TODO Auto-generated method stub
        super.reset(value);
    }

}
