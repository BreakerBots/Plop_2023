// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics.vector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;

/**
 * represents a 2 dimentional vector, a uantity with bolth magnatude and direction. Here representd as a total magnatude, angular direction, 
 * x manatude component, and y magnatude component
 */
public class BreakerVector2 implements BreakerInterpolable<BreakerVector2> {
    private Rotation2d vectorRotation;
    private double magnitude;
    private double magnitudeX;
    private double magnitudeY;


     /** creates an empty BreakerVector2 with 0's for all values */
     public BreakerVector2() {
        magnitudeX = 0;
           magnitudeY = 0;
           vectorRotation = Rotation2d.fromDegrees(0);
           magnitude = 0;
       }

    /**
     * creates a new BreakerVector2 from the magnatudes of the vector's X and Y
     * components
     */
    public BreakerVector2(double magnitudeX, double magnitudeY) {
        this.magnitudeX = magnitudeX;
        this.magnitudeY = magnitudeY;
        vectorRotation = new Rotation2d(Math.atan2(magnitudeY, magnitudeX));
        magnitude = Math.sqrt((magnitudeX * magnitudeX) + (magnitudeY * magnitudeY));
    }

    /**
     * creates a new BreakerVector2 from the vectors Magnatude and the vectors angle
     * in the Yaw angular axis
     */
    public BreakerVector2(Rotation2d vectorRotation, double magnatude) {
        this(magnatude * vectorRotation.getCos(),
                magnatude * vectorRotation.getSin(), magnatude, vectorRotation);
    }

    /** converts an instance of WPILib's translation2d class into a vector.
     *  This exists because of the Tranlation2d classes suppport of various vector opperations */
    public BreakerVector2(Translation2d translationToVectorize) {
       this(translationToVectorize.getX(), translationToVectorize.getY(), translationToVectorize.getNorm(), translationToVectorize.getAngle());
    }

    private BreakerVector2(double magnitudeX, double magnitudeY, double magnitude, Rotation2d vectorRotation) {
        this.magnitudeX = magnitudeX;
        this.magnitudeY = magnitudeY;
        this.magnitude = magnitude;
        this.vectorRotation = vectorRotation;
    }

    
    /** 
     * @return double
     */
    public double getMagnitude() {
        return magnitude;
    }

    
    /** 
     * @return Rotation2d
     */
    public Rotation2d getVectorRotation() {
        return vectorRotation;
    }

    
    /** 
     * @return double
     */
    public double getX() {
        return magnitudeX;
    }

    
    /** 
     * @return double
     */
    public double getY() {
        return magnitudeY;
    }

    
    /** 
     * @param outher
     * @return BreakerVector2
     */
    public BreakerVector2 plus(BreakerVector2 outher) {
        return new BreakerVector2(magnitudeX + outher.magnitudeX, magnitudeY + outher.magnitudeY);
    }

    
    /** 
     * @param outher
     * @return BreakerVector2
     */
    public BreakerVector2 minus(BreakerVector2 outher) {
        return new BreakerVector2(magnitudeX - outher.magnitudeX, magnitudeY - outher.magnitudeY);
    }

    
    /** 
     * @return BreakerVector2
     */
    public BreakerVector2 unaryMinus()  {
        return new BreakerVector2(-magnitudeX, -magnitudeY);
    }

    
    /** 
     * @param scalar
     * @return BreakerVector2
     */
    public BreakerVector2 times(double scalar) {
        return new BreakerVector2(magnitudeX * scalar, magnitudeY * scalar);
    }

    
    /** 
     * @param scalar
     * @return BreakerVector2
     */
    public BreakerVector2 div(double scalar) {
        return new BreakerVector2(magnitudeX / scalar,  magnitudeY / scalar);
    }
    
    
    /** 
     * @return Translation2d
     */
    public Translation2d getAsTranslation() {
        return new Translation2d(magnitudeX, magnitudeY);
    }

    
    /** 
     * @param rotation
     * @return BreakerVector2
     */
    public BreakerVector2 rotateBy(Rotation2d rotation) {
        double cos = Math.cos(rotation.getRadians());
        double sin = Math.sin(rotation.getRadians());
        return new BreakerVector2((this.magnitudeX * cos) - (this.magnitudeY * sin), (this.magnitudeX * sin) + (this.magnitudeY * cos));
    }

    
    /** 
     * @return BreakerVector2
     */
    public BreakerVector2 getUnitVector() {
        return new BreakerVector2(vectorRotation, 1.0);
    }

    
    /** 
     * @param obj
     * @return boolean
     */
    @Override
    public boolean equals(Object obj) {
        return (Math.abs(((BreakerVector2) obj).magnitudeX - magnitudeX) < 1E-9)
                && (Math.abs(((BreakerVector2) obj).magnitudeY - magnitudeY) < 1E-9);
    }

    
    /** 
     * @param endValue
     * @param t
     * @return BreakerVector2
     */
    @Override
    public BreakerVector2 interpolate(BreakerVector2 endValue, double t) {
        double interX = MathUtil.interpolate(magnitudeX, endValue.getX(), t);
        double interY = MathUtil.interpolate(magnitudeY, endValue.getY(), t);
        return new BreakerVector2(interX, interY);
    }

    /** [0] = X, [1] = Y */
    @Override
    public double[] getInterpolatableData() {
        return new double[] { magnitudeX, magnitudeY };
    }

    
    /** 
     * @param interpolatableData
     * @return BreakerVector2
     */
    @Override
    public BreakerVector2 fromInterpolatableData(double[] interpolatableData) {
        return new BreakerVector2(interpolatableData[0], interpolatableData[1]);
    }

    
    /** 
     * @return String
     */
    @Override
    public String toString() {
       return String.format("BreakerVector2(Vector_Magnatude: %.2f, X-Magnatude: %.2f, Y-Magnatude: %.2f,  Vector_Angle: %s)", magnitude, magnitudeX, magnitudeY, vectorRotation.toString());
    }

}
