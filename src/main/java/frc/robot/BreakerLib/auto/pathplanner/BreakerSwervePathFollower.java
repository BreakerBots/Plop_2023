package frc.robot.BreakerLib.auto.pathplanner;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveBase;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveMovementPreferences;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveMovementPreferences.SwerveMovementRefrenceFrame;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

/** Custom BreakerLib version of PathPlanner's custom version of SwerveControllerCommand */
public class BreakerSwervePathFollower extends CommandBase {
  private final Timer timer = new Timer();
  private final PathPlannerTrajectory trajectory;
  private PathPlannerTrajectory transformedTrajectory;
  private final BreakerSwervePathFollowerConfig config;
  private final boolean stopAtEnd;
  private final BreakerSwerveMovementPreferences driveMoveCallPrefrences;

  private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
  private static Consumer<Pose2d> logTargetPose = null;
  private static Consumer<ChassisSpeeds> logSetpoint = null;
  private static BiConsumer<Translation2d, Rotation2d> logError =
      BreakerSwervePathFollower::defaultLogError;

  /**
   * Constructs a new BreakerSwervePathFollower that when executed will follow the provided
   * trajectory.
   *
   * @param config 
   * @param trajectory The trajectory to follow.
   * @param stopAtEnd Weather or not the follower should stop the drivetrain upon completion of the path
   */
  public BreakerSwervePathFollower(
    BreakerSwervePathFollowerConfig config, 
    PathPlannerTrajectory trajectory,
    boolean stopAtEnd
    ) {
    this.trajectory = trajectory;
    this.config = config;
    this.stopAtEnd = stopAtEnd;
    driveMoveCallPrefrences = new BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITHOUT_OFFSET, SlowModeValue.DISABLED);
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
    if (config.getUseAllianceColor() && trajectory.fromGUI) {
      transformedTrajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = trajectory;
    }

    if (logActiveTrajectory != null) {
      logActiveTrajectory.accept(transformedTrajectory);
    }

    timer.reset();
    timer.start();

    PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    PathPlannerState desiredState = (PathPlannerState) transformedTrajectory.sample(currentTime);

    Pose2d currentPose = this.config.getDrivetrain().getOdometryPoseMeters();

    PathPlannerServer.sendPathFollowingData(
        new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
        currentPose);

    ChassisSpeeds targetChassisSpeeds = this.config.driveController.calculate(currentPose, desiredState);

    this.config.drivetrain.move(targetChassisSpeeds, driveMoveCallPrefrences);

    if (logTargetPose != null) {
      logTargetPose.accept(
          new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation));
    }

    if (logError != null) {
      logError.accept(
          currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
          currentPose.getRotation().minus(desiredState.holonomicRotation));
    }

    if (logSetpoint != null) {
      logSetpoint.accept(targetChassisSpeeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();

    if (interrupted || Math.abs(transformedTrajectory.getEndState().velocityMetersPerSecond) < 0.1 || stopAtEnd) {
      this.config.drivetrain.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
  }

  private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
    SmartDashboard.putNumber("BreakerSwervePathFollower/xErrorMeters", translationError.getX());
    SmartDashboard.putNumber("BreakerSwervePathFollower/yErrorMeters", translationError.getY());
    SmartDashboard.putNumber(
        "BreakerSwervePathFollower/rotationErrorDegrees", rotationError.getDegrees());
  }

  /**
   * Set custom logging callbacks for this command to use instead of the default configuration of
   * pushing values to SmartDashboard
   *
   * @param logActiveTrajectory Consumer that accepts a PathPlannerTrajectory representing the
   *     active path. This will be called whenever a PPSwerveControllerCommand starts
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
    BreakerSwervePathFollower.logActiveTrajectory = logActiveTrajectory;
    BreakerSwervePathFollower.logTargetPose = logTargetPose;
    BreakerSwervePathFollower.logSetpoint = logSetpoint;
    BreakerSwervePathFollower.logError = logError;
  }

  public static class BreakerSwervePathFollowerConfig {
    private BreakerSwerveDrive drivetrain;
    private PPHolonomicDriveController driveController;
    private boolean useAllianceColor;

    public BreakerSwervePathFollowerConfig(BreakerSwerveDrive drivetrain, PPHolonomicDriveController driveController, boolean useAllianceColor) {
      this.driveController = driveController;
      this.drivetrain = drivetrain;
      this.useAllianceColor = useAllianceColor;
    }

    public PPHolonomicDriveController getDriveController() {
        return driveController;
    }

    public BreakerSwerveDrive getDrivetrain() {
        return drivetrain;
    }

    public boolean getUseAllianceColor() {
        return useAllianceColor;
    }
  }
}



