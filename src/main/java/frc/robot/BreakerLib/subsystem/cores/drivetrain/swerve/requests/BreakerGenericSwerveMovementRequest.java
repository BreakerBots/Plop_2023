package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveRequest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.SwerveMovementRefrenceFrame;

public abstract class BreakerGenericSwerveMovementRequest implements BreakerSwerveRequest {
    protected SwerveMovementRefrenceFrame swerveMovementRefrenceFrame;
    protected SlowModeValue slowModeValue;
    protected boolean headingCorrectionEnabled;
    protected Translation2d centerOfRotation;
    protected boolean isOpenLoop;
    protected BreakerGenericSwerveMovementRequest(SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation, boolean isOpenLoop, boolean headingCorrectionEnabled) {
      this.slowModeValue = slowModeValue;
      this.swerveMovementRefrenceFrame = movementRefrenceFrame;
      this.centerOfRotation = centerOfRotation;
      this.headingCorrectionEnabled = headingCorrectionEnabled;
      this.isOpenLoop = isOpenLoop;
    } 

    public abstract BreakerGenericSwerveMovementRequest withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame swerveMovementRefrenceFrame);

    /** Sets whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
     * @param slowModeValue Whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
     * @return this.
     */
    public abstract BreakerGenericSwerveMovementRequest withSlowModeValue(SlowModeValue slowModeValue);

    public abstract BreakerGenericSwerveMovementRequest withCenerOfRotation(Translation2d centerOfRotation);

    public abstract BreakerGenericSwerveMovementRequest withIsOpenLoop(boolean isOpenLoop);

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
     applyChassisSpeeds(drivetrain, getRequestedChassisSpeeds(drivetrain), swerveMovementRefrenceFrame, slowModeValue, centerOfRotation, isOpenLoop, headingCorrectionEnabled); 
    }
  }