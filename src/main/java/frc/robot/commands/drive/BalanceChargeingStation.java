// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.subsystems.Drive;

public class BalanceChargeingStation extends CommandBase {
  /** Creates a new OffseasionBotBalanceChargeingStation. */
  private Drive drive;
  private BreakerPigeon2 imu;
  private boolean endOnBalance, enableTimeout;
  private final Timer timer = new Timer();
  public BalanceChargeingStation(Drive drive, BreakerPigeon2 imu, boolean endOnBalance, boolean enableTimeout) {
    this.drive = drive;
    this.imu = imu;
    this.enableTimeout = enableTimeout;
    this.endOnBalance = endOnBalance;
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    BreakerLog.logEvent("BalanceChargingStation command instance started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double robotRelX = DriveConstants.BALANCE_PITCH_PID.calculate(imu.getPitch(),  0.0);
    double robotRelY = DriveConstants.BALANCE_ROLL_PID.calculate(imu.getRoll(), 0.0);
    ChassisSpeeds robotRelSpeeds = new ChassisSpeeds(robotRelX, robotRelY, 0.0);
    ChassisSpeeds fieldRelSpds = BreakerMath.fromRobotRelativeSpeeds(robotRelSpeeds, drive.getOdometryPoseMeters().getRotation());
    fieldRelSpds.vyMetersPerSecond = 0.0;
    drive.move(fieldRelSpds, DriveConstants.AUTO_BALANCE_MOVEMENT_PREFERENCES);
    if (timer.hasElapsed(DriveConstants.AUTO_BALANCE_TIMEOUT_SEC)) {
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    if (interrupted) {
      BreakerLog.logEvent("BalanceChargingStation command instance TIMED OUT or was CANCLED, command did not neccicaraly fail");
    } else if (endOnBalance) {
      BreakerLog.logEvent("BalanceChargingStation command instance was SUCESSFULL, robot is balanced witn tolerence and command has ended");
    }
  }

  public boolean isBalanced() {
    boolean pitchPos = BreakerMath.epsilonEquals(imu.getPitch(), 0.0, DriveConstants.BALANCE_PITCH_POSITION_TOLERENCE);
    boolean rollPos = BreakerMath.epsilonEquals(imu.getRoll(), 0.0, DriveConstants.BALANCE_ROLL_POSITION_TOLERENCE);
    boolean pitchVel = BreakerMath.epsilonEquals(imu.getPitchRate(), 0.0, DriveConstants.BALANCE_PITCH_VELOSITY_TOLERENCE);
    boolean rollVel = BreakerMath.epsilonEquals(imu.getRollRate(), 0.0, DriveConstants.BALANCE_ROLL_VELOSITY_TOLERENCE);
    return pitchPos && rollPos && pitchVel && rollVel;
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endOnBalance && isBalanced();
  }
}
