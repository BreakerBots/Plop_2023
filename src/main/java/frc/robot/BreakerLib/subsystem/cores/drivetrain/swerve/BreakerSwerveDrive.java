// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveDriveFusedVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveRequest.BreakerGenericSwerveMovementRequest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveRequest.BreakerSwerveStopRequest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLogUtil;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.suites.BreakerGenericTestSuiteImplementation;
import frc.robot.BreakerLib.util.test.suites.drivetrain.swerve.BreakerSwerveDriveTestSuite;

public class BreakerSwerveDrive extends BreakerGenericDrivetrain implements BreakerGenericTestSuiteImplementation<BreakerSwerveDriveTestSuite> {
  /** Creates a new BreakerSwerveDrive2. */
  /**
   * The current {@link SwerveModuleState} each of this drivetrain's swerve
   * modules is set to
   */
  private SwerveModuleState[] targetModuleStates;

  /**
   * Each of the {@link BreakerGenericSwerveModule} instances controlled by this
   * class.
   */
  private BreakerGenericSwerveModule[] swerveModules;

  /**
   * The {@link BreakerGenericGyro} used for this drivetrain's internal odometery.
   */
  private BreakerGenericGyro gyro;

  /**
   * The {@link BreakerGenericOdometer} object this drivetrain uses for its internal
   * odometry.
   */
  private BreakerGenericOdometer odometer;

  private BreakerSwerveDriveConfig config;

  private SwerveDriveKinematics kinematics;

  private BreakerSwerveStopRequest stopRequest;

  private Rotation2d fieldRelativeMovementOffset = new Rotation2d();


  public BreakerSwerveDrive(
    BreakerSwerveDriveConfig config, BreakerGenericGyro gyro, 
    BreakerGenericSwerveModule... swerveModules) {
    this(config, new BreakerSwerveOdometryConfig(), gyro, swerveModules);
  }

  public BreakerSwerveDrive(
    BreakerSwerveDriveConfig config, BreakerSwerveOdometryConfig odometryConfig, 
    BreakerGenericGyro gyro, BreakerGenericSwerveModule... swerveModules) {
    this.config = config;
    this.swerveModules = swerveModules;
    this.gyro = gyro;
    deviceName = "Swerve_Drivetrain";
    targetModuleStates = new SwerveModuleState[swerveModules.length];
    Translation2d[] wheelPositions = new Translation2d[swerveModules.length];
    for (int i = 0; i < targetModuleStates.length; i++) {
      targetModuleStates[i] = new SwerveModuleState();
      wheelPositions[i] = swerveModules[i].getWheelPositionRelativeToRobot();
    }
    kinematics = new SwerveDriveKinematics(wheelPositions);
    odometer = odometryConfig.getOdometer(this);
    stopRequest = new BreakerSwerveStopRequest();
  }

  // /**
  //  * Directly exposes full module controll to the user, does not proform any
  //  * automatic optimization or desaturation
  //  * <p>
  //  * NOTE: Not affected by slow mode, does not consider technical limits of
  //  * modules or drivetrain,
  //  */
  // public void setRawModuleStates(SwerveModuleState... targetModuleStates) {
  //   for (int i = 0; i < swerveModules.length; i++) {
  //     swerveModules[i].setModuleTarget(targetModuleStates[i]);
  //     this.targetModuleStates[i] = targetModuleStates[i];
  //   }
  // }

  // /**
  //  * Sets each module to match a target module state in the order they were passed
  //  * in. Automaticly optimizes and desaturates wheelspeeds, stops module if set
  //  * speed is below module speed deadband threshold
  //  * <p>
  //  * NOTE: Not affected by slow mode.
  //  */
  // public void setModuleStates(SwerveModuleState... targetModuleStates) {
  //   SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleStates, config.getMaxAttainableModuleWheelSpeed());
  //   for (int i = 0; i < swerveModules.length; i++) {

  //     if (Math.abs(targetModuleStates[i].speedMetersPerSecond) < config.getModuleWheelSpeedDeadband()) {

  //       swerveModules[i].stop();

  //       this.targetModuleStates[i] = swerveModules[i].getModuleTargetState();

  //     } else {

  //       SwerveModuleState optimizedState = SwerveModuleState.optimize(targetModuleStates[i],
  //           swerveModules[i].getModuleAbsoluteAngle());

  //       swerveModules[i].setModuleTarget(optimizedState);
  //       this.targetModuleStates[i] = optimizedState;
  //     }

  //   }
  // }

  protected void setModuleStates(boolean applyRawModuleStates, SwerveModuleState... targetModuleStates) {
    if (applyRawModuleStates) {
      for (int i = 0; i < swerveModules.length; i++) {
        swerveModules[i].setModuleTarget(targetModuleStates[i]);
        this.targetModuleStates[i] = targetModuleStates[i];
      }
    } else {
      SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleStates, config.getMaxAttainableModuleWheelSpeed());
      for (int i = 0; i < swerveModules.length; i++) {

        if (Math.abs(targetModuleStates[i].speedMetersPerSecond) < config.getModuleWheelSpeedDeadband()) {

          swerveModules[i].stop();

          this.targetModuleStates[i] = swerveModules[i].getModuleTargetState();

        } else {

          SwerveModuleState optimizedState = SwerveModuleState.optimize(targetModuleStates[i],
              swerveModules[i].getModuleAbsoluteAngle());

          swerveModules[i].setModuleTarget(optimizedState);
          this.targetModuleStates[i] = optimizedState;
        }

      }
    }
    
  }

  protected void move(ChassisSpeeds targetChassisSpeeds, SwerveMovementRefrenceFrame swerveMovementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation, boolean headingCorrectionEnabled) {
    ChassisSpeeds targetVels = targetChassisSpeeds;
    Rotation2d curAng = odometer.getOdometryPoseMeters().getRotation();

    switch(swerveMovementRefrenceFrame) {
        case FIELD_RELATIVE_WITHOUT_OFFSET:
            targetVels = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds, curAng);
            break;
        case FIELD_RELATIVE_WITH_OFFSET:
            targetVels = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds,
                curAng.plus(fieldRelativeMovementOffset));
            break;
        case ROBOT_RELATIVE:
        default:
            break;
    }

    if (slowModeValue  == SlowModeValue.ENABLED || (slowModeValue == SlowModeValue.DEFAULT && slowModeActive)) {
        targetVels.vxMetersPerSecond *= config.getSlowModeLinearMultiplier();
        targetVels.vyMetersPerSecond *= config.getSlowModeLinearMultiplier();
        targetVels.omegaRadiansPerSecond *= config.getSlowModeTurnMultiplier();
    }

    setModuleStates(false, getKinematics().toSwerveModuleStates(targetVels, centerOfRotation));
  }

  public void applyRequest(BreakerSwerveRequest requestToApply) {
    requestToApply.apply(this);
  }


  // /** 
  //  * The primary method for controling this drivetrain. Takes in a {@link ChassisSpeeds} object, representing the robot's velocites 
  //  * in the X, Y, and Theta, axies relative to your chosen refrence frame. It then converts these into individual velocieies 
  //  * and headings for each module based on the prefrences you have provided.
  //  * 
  //  * @param targetChassisSpeeds   The commanded linear (m/s) and angular (rad/s) velocites of the robot. In robot-relative mode, 
  //  *                              positive x is torwards the bow (front) and positive y is torwards port (left) of the robot. 
  //  *                              In field-relative mode, torwards X is twords to your odometry heading's zero point, and posiive y 
  //  *                              is 90 degrees offset from that and to the left. If you are using field-relative mode with an angle
  //  *                              offset (must be bolth set using {@link BreakerSwerveDrive#setFieldRelativeMovementOffsetAngle} 
  //  *                              and the proper refrence frame must be selected in your prefrences) the offset angle will be added
  //  *                              to your odometry reading when transforming the chassis velosity vector from the field refrence 
  //  *                              frame to the robot refrence frame. If SlowMode is enabled in your prefrences, or your prefrences 
  //  *                              refer to the drivetrain's global default which happens to be enabled when this method is called,
  //  *                              the velocitys you set in each axis are multiplyd by the SlowMode multipler set in this drivetrain's config.
  //  * 
  //  * @param movementPreferences   The {@link BreakerSwerveMovementPreferences} object that contains the prefrences for this call.
  //  *                              NOTE: THIS CLASS ONLY RESPECTS THE PREFRENCES FOUND IN THE {@link BreakerSwerveMovementPreferences}
  //  *                              CLASS'S PUBLIC CONSTRUCTORS! Values such as {@link BreakerSwerveMovementPreferences#getHeadingCorrectionEnabled()} 
  //  *                              and others like it are not acounted for or supported by this method, and exist to enshure fallback compatablity between
  //  *                              this class's movement methods and the child prefrence classes such as {@link BreakerSwerveDriveBaseMovementPreferences}
  //  *                              used by inheritng classes like {@link BreakerSwerveDriveBase} that support more features, and vice-versa.
  //  */

  // public void move(ChassisSpeeds targetChassisSpeeds, BreakerSwerveMovementPreferences movementPreferences) {
  //   ChassisSpeeds targetVels = targetChassisSpeeds;
  //   Rotation2d curAng = odometer.getOdometryPoseMeters().getRotation();

  //   switch(movementPreferences.getSwerveMovementRefrenceFrame()) {
  //       case FIELD_RELATIVE_WITHOUT_OFFSET:
  //           targetVels = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds, curAng);
  //           break;
  //       case FIELD_RELATIVE_WITH_OFFSET:
  //           targetVels = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds,
  //               curAng.plus(fieldRelativeMovementOffset));
  //           break;
  //       case ROBOT_RELATIVE:
  //       default:
  //           break;
  //   }

  //   if (movementPreferences.getSlowModeValue() == SlowModeValue.ENABLED || (movementPreferences.getSlowModeValue() == SlowModeValue.DEFAULT && slowModeActive)) {
  //       targetVels.vxMetersPerSecond *= config.getSlowModeLinearMultiplier();
  //       targetVels.vyMetersPerSecond *= config.getSlowModeLinearMultiplier();
  //       targetVels.omegaRadiansPerSecond *= config.getSlowModeTurnMultiplier();
  //   }

  //   setModuleStates(getKinematics().toSwerveModuleStates(targetVels));
  // }


  // /** 
  //  * The primary method for controling this drivetrain. Takes in a {@link ChassisSpeeds} object, representing the robot's velocites 
  //  * in the X, Y, and Theta, axies. This method defaults to useing a field relative velociy refrence frame with the field relative
  //  * angle offset applyed if set. SlowMode is set to the drivetrain's global default. 
  //  * 
  //  * @param targetChassisSpeeds   The commanded linear (m/s) and angular (rad/s) velocites of the robot. In robot-relative mode, 
  //  *                              positive x is torwards the bow (front) and positive y is torwards port (left) of the robot. 
  //  *                              In field-relative mode, torwards X is twords to your odometry heading's zero point, and posiive y 
  //  *                              is 90 degrees offset from that and to the left. If you are using field-relative mode with an angle
  //  *                              offset (must be set using {@link BreakerSwerveDrive#setFieldRelativeMovementOffsetAngle}) the offset 
  //  *                              angle will be added to your odometry reading when transfrming the chassis velosity vector from the  
  //  *                              field refrence frame to the robot refrence frame. If SlowMode is enabled globaly for your drivetrain 
  //  *                              the velocitys you set in each axis are multiplyd by the SlowMode multipler set in this drivetrain's config.
  //  */
  // public void move(ChassisSpeeds targetChassisSpeeds) {
  //   move(targetChassisSpeeds, BreakerSwerveMovementPreferences.DEFAULT_FIELD_RELATIVE_PREFERENCES);
  // }

  // public void move(double percentX, double percentY, double percentOmega, BreakerSwerveMovementPreferences movementPreferences) {
  //   move(percentsToChassisSpeeds(percentX, percentY, percentOmega), movementPreferences);
  // } 

  // public void move(double percentX, double percentY, double percentOmega) {
  //   move(percentX, percentY, percentOmega, BreakerSwerveMovementPreferences.DEFAULT_FIELD_RELATIVE_PREFERENCES);
  // } 

  // protected ChassisSpeeds percentsToChassisSpeeds(double percentX, double percentY, double percentOmega) {
  //   double clampedX = MathUtil.clamp(percentX, -1.0, 1.0);
  //   double clampedY = MathUtil.clamp(percentY, -1.0, 1.0);
  //   double clampedOmega = MathUtil.clamp(percentOmega, -1.0, 1.0);
  //   BreakerVector2 linearVelVec = new BreakerVector2(clampedX, clampedY).times(config.getMaxLinearVel());
  //   return new ChassisSpeeds(linearVelVec.getX(), linearVelVec.getY(), clampedOmega * config.getMaxAngleVel());
  // }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public BreakerSwerveDriveConfig getConfig() {
      return config;
  }

  /** @return States of swerve modules. */
  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      moduleStates[i] = swerveModules[i].getModuleState();
    }
    return moduleStates;
  }
  
  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] moduleStates = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      moduleStates[i] = swerveModules[i].getModulePosition();
    }
    return moduleStates;
  }

  public void setFieldRelativeMovementOffsetAngle(Rotation2d fieldRelativeMovementOffset) {
    this.fieldRelativeMovementOffset = fieldRelativeMovementOffset;
  }

  public Rotation2d getFieldRelativeMovementOffsetAngle() {
    return fieldRelativeMovementOffset;
  }
  
  public void resetSwerveModuleDriveDistances() {
    for (BreakerGenericSwerveModule mod : swerveModules) {
      mod.resetModuleDriveEncoderPosition();
    }
  }

  public void setOdometer(BreakerGenericOdometer odometer) {
      this.odometer = odometer;
  }

  @Override
  public void setOdometryPosition(Pose2d newPose) {
    odometer.setOdometryPosition(newPose);
    
  }

  @Override
  public Pose2d getOdometryPoseMeters() {
    return odometer.getOdometryPoseMeters();
  }

  @Override
  public BreakerMovementState2d getMovementState() {
    return odometer.getMovementState();
  }

  @Override
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return odometer.getRobotRelativeChassisSpeeds();
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return odometer.getFieldRelativeChassisSpeeds();
  }

  @Override
  public void runSelfTest() {
    faultStr = "";
    health = DeviceHealth.NOMINAL;
    for (BreakerGenericSwerveModule module : swerveModules) {
      module.runSelfTest();
      if (module.hasFault()) {
        faultStr += " " + module.getDeviceName() + ": " + module.getFaults() + " ";
        health = health != DeviceHealth.INOPERABLE ? module.getHealth() : health;
      }
    }
  }

  @Override
  public BreakerSwerveDriveTestSuite getTestSuite() {
    return new BreakerSwerveDriveTestSuite(this, swerveModules);
  }


  @Override
  public BreakerGenericGyro getBaseGyro() {
    return gyro;
  }

  @Override
  public void setDrivetrainBrakeMode(boolean isEnabled) {
    for (BreakerGenericSwerveModule module : swerveModules) {
      if (RobotState.isEnabled()) {
        module.setDriveMotorBrakeMode(isEnabled);
        module.setTurnMotorBrakeMode(true);
      } else {
        module.setModuleBrakeMode(isEnabled);
      }
    }
  }

  public SwerveModuleState[] getTargetModuleStates() {
    return targetModuleStates;
  }


  @Override
  public void stop() {
   
  }

  @Override
  public String toString() {
    return String.format("BreakerSwerveDrive(Health: %s, Movement_State: %s, Swerve_Modules: %s)", health.toString(),
        odometer.getMovementState().toString(), Arrays.toString(swerveModules));
  }

  public ChassisSpeeds getTargetChassisSpeeds() {
      return kinematics.toChassisSpeeds(targetModuleStates);
  }

  @Override
  public void toLog(LogTable table) {
    table.put("DeviceHealth", getHealth().toString());
    table.put("RealModuleStates", BreakerLogUtil.formatSwerveModuleStateForLog(getSwerveModuleStates()));
    table.put("TargetModuleStates", BreakerLogUtil.formatSwerveModuleStateForLog(targetModuleStates));
    odometer.toLog(table.getSubtable("Odometry"));
    LogTable moduleTable = table.getSubtable("Modules");
    for (BreakerGenericSwerveModule module: swerveModules) {
      module.toLog(moduleTable.getSubtable(module.getDeviceName()));
    }
  }

  // public static class BreakerSwerveMovementPreferences {
  //   protected final SwerveMovementRefrenceFrame swerveMovementRefrenceFrame;
  //   protected final SlowModeValue slowModeValue;
  //   protected final boolean headingCorrectionEnabled;
  //   public static final BreakerSwerveMovementPreferences DEFAULT_ROBOT_RELATIVE_PREFERENCES = new BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame.ROBOT_RELATIVE, SlowModeValue.DEFAULT, false);
  //   public static final BreakerSwerveMovementPreferences DEFAULT_FIELD_RELATIVE_PREFERENCES = new BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET, SlowModeValue.DEFAULT, false);

  //   /** Uses the drivetrain as odometry provider and uses a field relative movement angle offset. */
  //   public BreakerSwerveMovementPreferences() {
  //       this(SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET, SlowModeValue.DEFAULT, false);
  //   }

  //   public BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue) {
  //       this(movementRefrenceFrame, slowModeValue, false);
  //   } 

  //   protected BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, boolean headingCorrectionEnabled) {
  //     this.slowModeValue = slowModeValue;
  //     this.swerveMovementRefrenceFrame = movementRefrenceFrame;
  //     this.headingCorrectionEnabled = headingCorrectionEnabled;
  //   } 

  //   public BreakerSwerveMovementPreferences withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame swerveMovementRefrenceFrame) {
  //       return new BreakerSwerveMovementPreferences(swerveMovementRefrenceFrame, this.slowModeValue, this.headingCorrectionEnabled);
  //   }

  //   /** Sets whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
  //    * @param slowModeValue Whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
  //    * @return this.
  //    */
  //   public BreakerSwerveMovementPreferences withSlowModeValue(SlowModeValue slowModeValue) {
  //       return new BreakerSwerveMovementPreferences(this.swerveMovementRefrenceFrame, slowModeValue, this.headingCorrectionEnabled);
  //   }

  //   public SlowModeValue getSlowModeValue() {
  //       return slowModeValue;
  //   }

  //   public SwerveMovementRefrenceFrame getSwerveMovementRefrenceFrame() {
  //       return swerveMovementRefrenceFrame;
  //   }

  //   public boolean getHeadingCorrectionEnabled() {
  //       return headingCorrectionEnabled;
  //   }

  //   public enum SwerveMovementRefrenceFrame {
  //       FIELD_RELATIVE_WITH_OFFSET,
  //       FIELD_RELATIVE_WITHOUT_OFFSET,
  //       ROBOT_RELATIVE
  //   }
  // }

  public enum SwerveMovementRefrenceFrame {
    FIELD_RELATIVE_WITH_OFFSET,
    FIELD_RELATIVE_WITHOUT_OFFSET,
    ROBOT_RELATIVE
  }

  public static class BreakerSwerveOdometryConfig {
    private BreakerGenericVisionOdometer vision;
    private Pose2d initalPose;
    private double[] stateStanderdDeveation, visionStanderdDeveation;
    private boolean usePoseEstimator;

    public BreakerSwerveOdometryConfig() {
       this(new Pose2d());
    }

    public BreakerSwerveOdometryConfig(Pose2d initalPose) {
        this.initalPose = initalPose;
        usePoseEstimator = false;
    }

    public BreakerSwerveOdometryConfig(
        BreakerGenericVisionOdometer vision,
        Pose2d initalPose,
        double[] stateStanderdDeveation,
        double[] visionStanderdDeveation
        ) {
       this.initalPose = initalPose;
       this.vision = vision;
       this.stateStanderdDeveation = stateStanderdDeveation;
       this.visionStanderdDeveation = visionStanderdDeveation;
       usePoseEstimator = true;
    }

    public BreakerSwerveOdometryConfig(
        BreakerGenericVisionOdometer vision,
        double[] stateStanderdDeveation,
        double[] visionStanderdDeveation
        ) {
        this(vision, new Pose2d(), stateStanderdDeveation, visionStanderdDeveation);
    }

    public BreakerGenericOdometer getOdometer(BreakerSwerveDrive drivetrain) {
        if (usePoseEstimator) {
            return new BreakerSwerveDriveFusedVisionPoseEstimator(drivetrain, vision, initalPose, visionStanderdDeveation, stateStanderdDeveation);
        }
        return new BreakerSwerveOdometer(drivetrain, initalPose);
    }
  }

  public interface BreakerSwerveRequest {
    public abstract void apply(BreakerSwerveDrive drivetrain);

    public static abstract class BreakerGenericSwerveMovementRequest implements BreakerSwerveRequest {
      protected SwerveMovementRefrenceFrame swerveMovementRefrenceFrame;
      protected SlowModeValue slowModeValue;
      protected boolean headingCorrectionEnabled;
      protected Translation2d centerOfRotation;
      protected BreakerGenericSwerveMovementRequest(SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation, boolean headingCorrectionEnabled) {
        this.slowModeValue = slowModeValue;
        this.swerveMovementRefrenceFrame = movementRefrenceFrame;
        this.centerOfRotation = centerOfRotation;
        this.headingCorrectionEnabled = headingCorrectionEnabled;
      } 

      public abstract BreakerGenericSwerveMovementRequest withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame swerveMovementRefrenceFrame);

      /** Sets whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
       * @param slowModeValue Whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
       * @return this.
       */
      public abstract BreakerGenericSwerveMovementRequest withSlowModeValue(SlowModeValue slowModeValue);

      public abstract BreakerGenericSwerveMovementRequest withCenerOfRotation(Translation2d centerOfRotation);

      public SlowModeValue getSlowModeValue() {
          return slowModeValue;
      }

      public SwerveMovementRefrenceFrame getSwerveMovementRefrenceFrame() {
          return swerveMovementRefrenceFrame;
      }

      public Translation2d getCenterOfRotation() {
          return centerOfRotation;
      }

      public boolean getHeadingCorrectionEnabled() {
          return headingCorrectionEnabled;
      }

      public abstract ChassisSpeeds getRequestedChassisSpeeds(BreakerSwerveDrive drivetrain);

      @Override
      public void apply(BreakerSwerveDrive drivetrain) {
        drivetrain.move(getRequestedChassisSpeeds(drivetrain), swerveMovementRefrenceFrame, slowModeValue, centerOfRotation, headingCorrectionEnabled); 
      }
    }

    public static class BreakerSwerveVelocityRequest extends BreakerGenericSwerveMovementRequest {
      protected ChassisSpeeds speeds;
      public BreakerSwerveVelocityRequest(ChassisSpeeds speeds) {
        this(speeds, SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET, SlowModeValue.DEFAULT, new Translation2d(), false);
      }

      public BreakerSwerveVelocityRequest(ChassisSpeeds speeds, SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation) {
        this(speeds, movementRefrenceFrame, slowModeValue, centerOfRotation, false);
      }

      protected BreakerSwerveVelocityRequest(ChassisSpeeds speeds, SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation, boolean headingCorrectionEnabled) {
        super(movementRefrenceFrame, slowModeValue, centerOfRotation, headingCorrectionEnabled);
        this.speeds = speeds;
      }

      @Override
      public BreakerSwerveVelocityRequest withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame swerveMovementRefrenceFrame) {
        this.swerveMovementRefrenceFrame = swerveMovementRefrenceFrame;
        return this;
      }

      @Override
      public BreakerSwerveVelocityRequest withSlowModeValue(SlowModeValue slowModeValue) {
        this.slowModeValue = slowModeValue;
        return this;
      }

      public BreakerSwerveVelocityRequest withChassisSpeeds(ChassisSpeeds speeds) {
        this.speeds = speeds;
        return this;
      }

      @Override
      public BreakerSwerveVelocityRequest withCenerOfRotation(Translation2d centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
        return this;
      }

      public BreakerSwerveVelocityRequest withVelocityX(double velocityX) {
        speeds.vxMetersPerSecond = velocityX;
        return this;
      }

      public BreakerSwerveVelocityRequest withVelocityY(double velocityY) {
        speeds.vyMetersPerSecond = velocityY;
        return this;
      }

      public BreakerSwerveVelocityRequest withVelocityOmega(double velocityOmega) {
        speeds.omegaRadiansPerSecond = velocityOmega;
        return this;
      }

      public double getVelocityX() {
        return speeds.vxMetersPerSecond;
      }

      public double getVelocityY() {
        return speeds.vyMetersPerSecond;
      }

      public double getVelocityOmega() {
        return speeds.omegaRadiansPerSecond;
      }

      @Override
      public ChassisSpeeds getRequestedChassisSpeeds(BreakerSwerveDrive drivetrain) {
        return speeds;
      }
    }

    public static class BreakerSwervePercentSpeedRequest extends BreakerGenericSwerveMovementRequest {
      protected ChassisPercentSpeeds percentSpeeds;

      public BreakerSwervePercentSpeedRequest(ChassisPercentSpeeds percentSpeeds) {
        this(percentSpeeds, SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET, SlowModeValue.DEFAULT, new Translation2d(), false);
      }

      public BreakerSwervePercentSpeedRequest(ChassisPercentSpeeds percentSpeeds, SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation) {
        this(percentSpeeds, movementRefrenceFrame, slowModeValue, centerOfRotation, false);
      }

      protected BreakerSwervePercentSpeedRequest(ChassisPercentSpeeds percentSpeeds, SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation, boolean headingCorrectionEnabled) {
        super(movementRefrenceFrame, slowModeValue, centerOfRotation, headingCorrectionEnabled);
        this.percentSpeeds = percentSpeeds;
      }

      @Override
      public BreakerSwervePercentSpeedRequest withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame swerveMovementRefrenceFrame) {
        this.swerveMovementRefrenceFrame = swerveMovementRefrenceFrame;
        return this;
      }

      @Override
      public BreakerSwervePercentSpeedRequest withSlowModeValue(SlowModeValue slowModeValue) {
        this.slowModeValue = slowModeValue;
        return this;
      }

      @Override
      public BreakerSwervePercentSpeedRequest withCenerOfRotation(Translation2d centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
        return this;
      }

      public BreakerSwervePercentSpeedRequest withPercentX(double percentX) {
        percentSpeeds.vxPercentOfMax = MathUtil.clamp(percentX, -1.0, 1.0);
        return this;
      }

      public BreakerSwervePercentSpeedRequest withPercentY(double percentY) {
        percentSpeeds.vyPercentOfMax = MathUtil.clamp(percentY, -1.0, 1.0);
        return this;
      }

      public BreakerSwervePercentSpeedRequest withPercentOmega(double percentOmega) {
        percentSpeeds.omegaPercentOfMax = MathUtil.clamp(percentOmega, -1.0, 1.0);
        return this;
      }

      public BreakerSwervePercentSpeedRequest withChassisPercentSpeeds(ChassisPercentSpeeds percentSpeeds) {
        this.percentSpeeds = percentSpeeds;
        return this;
      }

      public double getPercentX() {
        return percentSpeeds.vxPercentOfMax;
      }

      public double getPercentY() {
        return percentSpeeds.vyPercentOfMax;
      }

      public double getPercentOmega() {
        return percentSpeeds.omegaPercentOfMax;
      }

      @Override
      public ChassisSpeeds getRequestedChassisSpeeds(BreakerSwerveDrive drivetrain) {
        return percentSpeeds.toChassisSpeeds(drivetrain.getConfig().getMaxLinearVel(), drivetrain.getConfig().getMaxAngleVel());
      }

      public static class ChassisPercentSpeeds {
        /** Represents forward velocity w.r.t the robot frame of reference. (Fwd is +) */
        public double vxPercentOfMax;
      
        /** Represents sideways velocity w.r.t the robot frame of reference. (Left is +) */
        public double vyPercentOfMax;
      
        /** Represents the angular velocity of the robot frame. (CCW is +) */
        public double omegaPercentOfMax;
      
        /** Constructs a ChassisSpeeds with zeros for dx, dy, and theta. */
        public ChassisPercentSpeeds() {}
      
        /**
         * Constructs a ChassisSpeeds object.
         *
         * @param vxPercentOfMax Forward velocity.
         * @param vyPercentOfMax Sideways velocity.
         * @param omegaPercentOfMax Angular velocity.
         */
        public ChassisPercentSpeeds(
            double vxPercentOfMax, double vyPercentOfMax, double omegaPercentOfMax) {
          this.vxPercentOfMax = MathUtil.clamp(vxPercentOfMax, -1.0, 1.0);
          this.vyPercentOfMax = MathUtil.clamp(vyPercentOfMax, -1.0, 1.0);
          this.omegaPercentOfMax = MathUtil.clamp(omegaPercentOfMax, -1.0, 1.0);
        }
      
        /**
         * Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds
         * object.
         *
         * @param vxPercentOfMax The component of speed in the x direction relative to the field.
         *     Positive x is away from your alliance wall.
         * @param vyPercentOfMax The component of speed in the y direction relative to the field.
         *     Positive y is to your left when standing behind the alliance wall.
         * @param omegaPercentOfMax The angular rate of the robot.
         * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
         *     considered to be zero when it is facing directly away from your alliance station wall.
         *     Remember that this should be CCW positive.
         * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
         */
        public static ChassisPercentSpeeds fromFieldRelativeSpeeds(
            double vxPercentOfMax,
            double vyPercentOfMax,
            double omegaPercentOfMax,
            Rotation2d robotAngle) {
          return new ChassisPercentSpeeds(
              MathUtil.clamp(vxPercentOfMax, -1.0, 1.0) * robotAngle.getCos() + MathUtil.clamp(vyPercentOfMax, -1.0, 1.0) * robotAngle.getSin(),
              -MathUtil.clamp(vxPercentOfMax, -1.0, 1.0) * robotAngle.getSin() + MathUtil.clamp(vyPercentOfMax, -1.0, 1.0) * robotAngle.getCos(),
              omegaPercentOfMax);
        }
      
        /**
         * Converts a user provided field-relative ChassisSpeeds object into a robot-relative
         * ChassisSpeeds object.
         *
         * @param fieldRelativeSpeeds The ChassisSpeeds object representing the speeds in the field frame
         *     of reference. Positive x is away from your alliance wall. Positive y is to your left when
         *     standing behind the alliance wall.
         * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
         *     considered to be zero when it is facing directly away from your alliance station wall.
         *     Remember that this should be CCW positive.
         * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
         */
        public static ChassisPercentSpeeds fromFieldRelativeSpeeds(
            ChassisPercentSpeeds fieldRelativeSpeeds, Rotation2d robotAngle) {
          return fromFieldRelativeSpeeds(
              fieldRelativeSpeeds.vxPercentOfMax,
              fieldRelativeSpeeds.vyPercentOfMax,
              fieldRelativeSpeeds.omegaPercentOfMax,
              robotAngle);
        }
      
        @Override
        public String toString() {
          return String.format(
              "ChassisSpeeds(Vx: %.2f (% of max), Vy: %.2f (% of max), Omega: %.2f (% of max))",
              vxPercentOfMax, vyPercentOfMax, omegaPercentOfMax);
        }

        public ChassisSpeeds toChassisSpeeds(double maxLinearVel, double maxAngVel) {
          double clampedX = MathUtil.clamp(vxPercentOfMax, -1.0, 1.0);
          double clampedY = MathUtil.clamp(vyPercentOfMax, -1.0, 1.0);
          double clampedOmega = MathUtil.clamp(omegaPercentOfMax, -1.0, 1.0);
          BreakerVector2 linearVelVec = new BreakerVector2(clampedX, clampedY).times(maxLinearVel);
          return new ChassisSpeeds(linearVelVec.getX(), linearVelVec.getY(), clampedOmega * maxAngVel);
        }
      }
    }

    public static class BreakerSwerveModuleStateRequest implements BreakerSwerveRequest {
      protected boolean applyRawStates;
      protected SwerveModuleState[] moduleStates;

      public BreakerSwerveModuleStateRequest(boolean applyRawStates, SwerveModuleState... moduleStates) {
        this.applyRawStates = applyRawStates;
        this.moduleStates = moduleStates;
      }

      public boolean getApplyRawStates() {
          return applyRawStates;
      }

      public SwerveModuleState[] getRequestedModuleStates() {
          return moduleStates;
      }

      public BreakerSwerveModuleStateRequest withApplyRawStates(boolean applyRawStates) {
        this.applyRawStates = applyRawStates;
        return this;
      }

      public BreakerSwerveModuleStateRequest withModuleStates(SwerveModuleState... moduleStates) {
        this.moduleStates = moduleStates;
        return this;
      }

      @Override
      public void apply(BreakerSwerveDrive drivetrain) {
        drivetrain.setModuleStates(applyRawStates, moduleStates);
      }
    }

    public static class BreakerSwerveAllignModulesToAngleRequest implements BreakerSwerveRequest {
      protected Rotation2d angle;
      public BreakerSwerveAllignModulesToAngleRequest(Rotation2d angle) {
        this.angle = angle;
      }

      @Override
      public void apply(BreakerSwerveDrive drivetrain) {
        SwerveModuleState[] requestStates = new SwerveModuleState[drivetrain.getSwerveModulePositions().length];
        for (int i = 0; i < drivetrain.getSwerveModulePositions().length; i++) {
          requestStates[i] = new SwerveModuleState(0.0, angle);
        }
        drivetrain.setModuleStates(false, requestStates);
      }
      
    }

    public static class BreakerSwerveStopRequest implements BreakerSwerveRequest {

      public BreakerSwerveStopRequest() {}

      @Override
      public void apply(BreakerSwerveDrive drivetrain) {
        drivetrain.move(new ChassisSpeeds(), SwerveMovementRefrenceFrame.ROBOT_RELATIVE, SlowModeValue.DEFAULT, new Translation2d(), false);
      }
    }
  }
}