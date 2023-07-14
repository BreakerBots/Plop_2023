package frc.robot.BreakerLib.auto.pathplanner;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDrive;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

/** Custom BreakerLib version of PathPlanner's custom version of RamseteCommand */
public class BreakerRamsetePathFollower extends CommandBase {
  private final Timer timer = new Timer();
  private final PathPlannerTrajectory trajectory;
  private final BreakerRamsetePathFollowerConfig config;
  private final boolean stopAtEnd;

  private DifferentialDriveWheelSpeeds prevSpeeds;
  private double prevTime;

  private PathPlannerTrajectory transformedTrajectory;

  private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
  private static Consumer<Pose2d> logTargetPose = null;
  private static Consumer<ChassisSpeeds> logSetpoint = null;
  private static BiConsumer<Translation2d, Rotation2d> logError = BreakerRamsetePathFollower::defaultLogError;

  /**
   * Constructs a new BreakerRamsetePathFollower that, when executed, will follow the provided trajectory. PID
   * control and feedforward are handled internally.
   *
   * @param trajectory The trajectory to follow.
   */
  public BreakerRamsetePathFollower(
      BreakerRamsetePathFollowerConfig config,
      PathPlannerTrajectory trajectory,
      boolean stopAtEnd
    ) {
    this.trajectory = trajectory;
    this.config = config;
    this.stopAtEnd = stopAtEnd;
    addRequirements(config.getDrivetrain());

    if (config.getUseAllianceColor() && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
      DriverStation.reportWarning(
          "You have constructed a path following command that will automatically transform path states depending"
              + " on the alliance color, however, it appears this path was created on the red side of the field"
              + " instead of the blue side. This is likely an error.",
          false);
    }
  }

  @Override
  public void initialize() {
    if (this.config.getUseAllianceColor() && trajectory.fromGUI) {
      transformedTrajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = trajectory;
    }

    this.prevTime = -1;

    if (logActiveTrajectory != null) {
      logActiveTrajectory.accept(transformedTrajectory);
    }

    PathPlannerTrajectory.PathPlannerState initialState = transformedTrajectory.getInitialState();

    this.prevSpeeds =
        this.config.getDrivetrain().getKinematics().toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));

    this.timer.reset();
    this.timer.start();

    this.config.getLeftController().reset();
    this.config.getRightController().reset();

    PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    double dt = currentTime - this.prevTime;

    if (this.prevTime < 0) {
      this.prevTime = currentTime;
      return;
    }

    Pose2d currentPose = this.config.odometer.getOdometryPoseMeters();
    PathPlannerTrajectory.PathPlannerState desiredState =
        (PathPlannerTrajectory.PathPlannerState) transformedTrajectory.sample(currentTime);

    PathPlannerServer.sendPathFollowingData(desiredState.poseMeters, currentPose);

    ChassisSpeeds targetChassisSpeeds = this.config.getRamseteController().calculate(currentPose, desiredState);
    DifferentialDriveWheelSpeeds targetWheelSpeeds =
        this.config.getDrivetrain().getKinematics().toWheelSpeeds(targetChassisSpeeds);

    double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

      double leftFeedforward =
          this.config.getFeedforward().calculate(
              leftSpeedSetpoint, (leftSpeedSetpoint - this.prevSpeeds.leftMetersPerSecond) / dt);
      double rightFeedforward =
          this.config.getFeedforward().calculate(
              rightSpeedSetpoint, (rightSpeedSetpoint - this.prevSpeeds.rightMetersPerSecond) / dt);

      leftOutput =
          leftFeedforward
              + this.config.getLeftController().calculate(
                  this.config.getDrivetrain().getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint);
      rightOutput =
          rightFeedforward
              + this.config.getRightController().calculate(
                this.config.getDrivetrain().getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint);

    this.config.getDrivetrain().tankDriveVoltage(leftOutput, rightOutput);
    this.prevSpeeds = targetWheelSpeeds;
    this.prevTime = currentTime;

    if (logTargetPose != null) {
      logTargetPose.accept(desiredState.poseMeters);
    }

    if (logError != null) {
      logError.accept(
          currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
          currentPose.getRotation().minus(desiredState.poseMeters.getRotation()));
    }

    if (logSetpoint != null) {
      logSetpoint.accept(targetChassisSpeeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();

    if (interrupted
        || Math.abs(transformedTrajectory.getEndState().velocityMetersPerSecond) < 0.1 || stopAtEnd) {
      this.config.getDrivetrain().stop();
    }
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
  }

  private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
    SmartDashboard.putNumber("PPRamseteCommand/xErrorMeters", translationError.getX());
    SmartDashboard.putNumber("PPRamseteCommand/yErrorMeters", translationError.getY());
    SmartDashboard.putNumber("PPRamseteCommand/rotationErrorDegrees", rotationError.getDegrees());
  }

  /**
   * Set custom logging callbacks for this command to use instead of the default configuration of
   * pushing values to SmartDashboard
   *
   * @param logActiveTrajectory Consumer that accepts a PathPlannerTrajectory representing the
   *     active path. This will be called whenever a PPRamseteCommand starts
   * @param logTargetPose Consumer that accepts a Pose2d representing the target pose while path
   *     following
   * @param logSetpoint Consumer that accepts a ChassisSpeeds object representing the setpoint
   *     speeds
   * @param logError BiConsumer that accepts a Translation2d and Rotation2d representing the error
   *     while path following
   */
  public static void setLoggingCallbacks(
      Consumer<PathPlannerTrajectory> logActiveTrajectory,
      Consumer<Pose2d> logTargetPose,
      Consumer<ChassisSpeeds> logSetpoint,
      BiConsumer<Translation2d, Rotation2d> logError) {
    BreakerRamsetePathFollower.logActiveTrajectory = logActiveTrajectory;
    BreakerRamsetePathFollower.logTargetPose = logTargetPose;
    BreakerRamsetePathFollower.logSetpoint = logSetpoint;
    BreakerRamsetePathFollower.logError = logError;
  }

public static class BreakerRamsetePathFollowerConfig {
  private BreakerDiffDrive drivetrain;
  private BreakerGenericOdometer odometer;
  private RamseteController ramseteController;
  private SimpleMotorFeedforward feedforward;
  private PIDController leftController;
  private PIDController rightController;
  private boolean useAllianceColor;
  public BreakerRamsetePathFollowerConfig(
    BreakerDiffDrive drivetrain, 
    RamseteController ramseteController, 
    SimpleMotorFeedforward feedforward,  
    PIDController leftController,
    PIDController rightController,
    boolean useAllianceColor
    ) {
      this.ramseteController = ramseteController;
      this.drivetrain = drivetrain;
      this.feedforward = feedforward;
      this.leftController = leftController;
      this.rightController = rightController;
      this.useAllianceColor = useAllianceColor;
      odometer = drivetrain;
    }

    public BreakerRamsetePathFollowerConfig(
    BreakerDiffDrive drivetrain, 
    BreakerGenericOdometer odometer,
    RamseteController ramseteController, 
    SimpleMotorFeedforward feedforward,  
    PIDController leftController,
    PIDController rightController,
    boolean useAllianceColor
    ) {
      this.ramseteController = ramseteController;
      this.drivetrain = drivetrain;
      this.odometer = odometer;
      this.feedforward = feedforward;
      this.leftController = leftController;
      this.rightController = rightController;
      this.useAllianceColor = useAllianceColor;
    }

    public BreakerDiffDrive getDrivetrain() {
        return drivetrain;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public PIDController getLeftController() {
        return leftController;
    }

    public BreakerGenericOdometer getOdometer() {
        return odometer;
    }

    public RamseteController getRamseteController() {
        return ramseteController;
    }

    public PIDController getRightController() {
        return rightController;
    }

    public boolean getUseAllianceColor() {
        return useAllianceColor;
    }
}
}