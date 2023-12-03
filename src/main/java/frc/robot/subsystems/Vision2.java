// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;

/** Add your docs here. */
public class Vision2 implements BreakerGenericOdometer {

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        // TODO Auto-generated method stub
        return null;
    }}
