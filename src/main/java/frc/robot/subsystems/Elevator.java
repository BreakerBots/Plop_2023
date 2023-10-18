// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.driverstation.dashboard.BreakerDashboard;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLoggable;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.selftest.SystemDiagnostics;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MiscConstants;

/** Add your docs here. */
public class Elevator extends SubsystemBase implements BreakerLoggable {
    private final TalonFX leftMotor, rightMotor;

    private final MotionMagicDutyCycle motionMagicRequest;
    private final DutyCycleOut dutyCycleRequest;
    private final NeutralOut lockRequest;
    private final CoastOut neutralRequest;
    private final Follower followRequest;

    private final Supplier<ForwardLimitValue> forwardLimit;
    private final Supplier<ReverseLimitValue> reverseLimit;
    private final Supplier<Double> elevatorPositionMetersRaw, elevatorVelocity;

    private final SystemDiagnostics diagnostics;

    private ElevatorControlMode currentState = ElevatorControlMode.AUTOMATIC; //ElevatorControlMode.CALIBRATING;
    private boolean hasBeenCalibrated = false;
   
    private double targetHeightMeters;
    private double manualControlDutyCycle;

    //private ElevatorSimManager simManager;

    private boolean isForceStoped = false;
    private boolean bypassCalibrationRoutine = false;
    private boolean revLimitTriggeredPrevCycle = false;
    private boolean fwdLimitTriggeredPrevCycle = false;

    private CalibrationRoutine calibrationRoutine;

   
    public Elevator() {
        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, MiscConstants.CANIVORE_1);
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, MiscConstants.CANIVORE_1);
        
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();

        leftConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MOTION_MAGIC_CRUISE_VEL;
        leftConfig.MotionMagic.MotionMagicAcceleration= ElevatorConstants.MOTION_MAGIC_ACCEL;
        leftConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.MOTION_MAGIC_JERK;

        leftConfig.Slot0.kP = ElevatorConstants.PIDF_KP;
        leftConfig.Slot0.kI = ElevatorConstants.PIDF_KI;
        leftConfig.Slot0.kD = ElevatorConstants.PIDF_KD;
        leftConfig.Slot0.kS = ElevatorConstants.PIDF_KS;
        leftConfig.Slot0.kV = ElevatorConstants.PIDF_KV;

        leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leftConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.MOTOR_ROT_TO_METERS_RATIO;

        leftConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        leftConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        leftConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
        leftConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        leftConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        leftConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

       // leftConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = ElevatorConstants.MAX_HEIGHT_ROBOT_REL;
        //leftConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
       // leftConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = ElevatorConstants.MIN_HEIGHT_ROBOT_REL;
       // leftConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;

        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;//Break
        leftConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CUR_LIMIT;
        leftConfig.CurrentLimits.SupplyTimeThreshold = ElevatorConstants.SUPPLY_CUR_LIMIT_TIME;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        
        leftMotor.getConfigurator().apply(leftConfig);

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;//Break
        rightConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CUR_LIMIT;
        rightConfig.CurrentLimits.SupplyTimeThreshold = ElevatorConstants.SUPPLY_CUR_LIMIT_TIME;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightMotor.getConfigurator().apply(rightConfig);

        targetHeightMeters = ElevatorTargetState.STOW.getTargetHeight();

        motionMagicRequest = new MotionMagicDutyCycle(0, false, 0, 0, false);
        dutyCycleRequest = new DutyCycleOut(0, false, false);
        lockRequest = new NeutralOut();
        neutralRequest = new CoastOut();

        forwardLimit = leftMotor.getForwardLimit().asSupplier();
        reverseLimit = leftMotor.getReverseLimit().asSupplier();
        elevatorPositionMetersRaw = leftMotor.getPosition().asSupplier();
        elevatorVelocity = leftMotor.getVelocity().asSupplier();

        diagnostics = new SystemDiagnostics("Elevator");
        diagnostics.addPhoenix6TalonFXs(leftMotor, rightMotor);
        diagnostics.addSupplier(this::healthCheck);

        calibrationRoutine = new CalibrationRoutine();
       // simManager = this.new ElevatorSimManager();
       followRequest = new Follower(ElevatorConstants.LEFT_MOTOR_ID, false);
       rightMotor.setControl(followRequest);
       BreakerDashboard.getMainTab().add(this);
       BreakerLog.getInstance().registerLogable("Elevator", this);

       bypassCalibrationRoutine = false;
       revLimitTriggeredPrevCycle = false;
       fwdLimitTriggeredPrevCycle = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("ELEV FAULT", diagnostics::hasFault, null);
        builder.addDoubleProperty("ELEV HEIGHT", this::getHeight, null);
        builder.addDoubleProperty("ELEV TARGET", this::getTargetHeightMeters, null);
        builder.addDoubleProperty("ELEV VELOCITY", this::getVelocity, null);
        builder.addStringProperty("ELEV TGT STATE", this::getTargetStateString, null);
        builder.addStringProperty("ELEV CTRL MODE", () -> getCurrentControlMode().toString(), null);
    }

    private Pair<DeviceHealth, String> healthCheck() {
        DeviceHealth health = DeviceHealth.NOMINAL;
        String str = "";
        if (getForwardLimitTriggered() && getReverseLimitTriggered()) {
            str += " limit_switch_malfunction_fwd_&_rev_swiches_triggered ";
            health = DeviceHealth.INOPERABLE;
        }
        if (isForceStoped) {
            str += " force_stoped_elevator_inoporable ";
            health = DeviceHealth.INOPERABLE;
        }
        return new Pair<DeviceHealth,String>(health, str);
    }
 
    private String getTargetStateString() {
       Optional<ElevatorTargetState> tgtOpt = ElevatorTargetState.getTargetFromHeight(getTargetHeightMeters());
       if (tgtOpt.isPresent()) {
            return tgtOpt.get().toString();
       }
       return "UNKNOWN";
    }
 
    public ElevatorControlMode getCurrentControlMode() {
        return currentState;
    }

    public void setTarget(double heightMeters) {
        targetHeightMeters = heightMeters;
        currentState = ElevatorControlMode.AUTOMATIC;
    }

    public void setManual(double dutyCycle) {
        manualControlDutyCycle = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        currentState = ElevatorControlMode.MANUAL;
    }

    public void calibrate() {
        currentState = ElevatorControlMode.CALIBRATING;
    }

    public double getHeight() {
        return elevatorPositionMetersRaw.get() + ElevatorConstants.MIN_HEIGHT_GND_REL;
    }

    public double getVelocity() {
        return elevatorVelocity.get();
    }

    public boolean atTargetHeight() {
        return BreakerMath.epsilonEquals(getHeight(), targetHeightMeters, ElevatorConstants.HIGHT_TOLARENCE) && currentState == ElevatorControlMode.AUTOMATIC;
    }

    public double getTargetHeightMeters() {
        return targetHeightMeters;
    }

    public boolean getForwardLimitTriggered() {
        return forwardLimit.get() == ForwardLimitValue.ClosedToGround;
    }

    public boolean getReverseLimitTriggered() {
        return reverseLimit.get() == ReverseLimitValue.ClosedToGround;
    }

    public void setNeutral() {
        currentState = ElevatorControlMode.NEUTRAL;
    }

    public void setLocked() {
        currentState = ElevatorControlMode.LOCKED;
    }

    /** */
    public void forceStop() {
        isForceStoped = true;
        BreakerLog.getInstance().logEvent("ELEVATOR FORCE STOPED - Elevator now unresponsive and locked");
    }

    public void endForceStop() {
        isForceStoped = false;
        BreakerLog.getInstance().logEvent("ELEVATOR FORCE STOP ENDED - Elevator now responsive and functional");
    }

    public boolean isForceStoped() {
        return isForceStoped;
    }

    @Override
    public void simulationPeriodic() {
        //simManager.updateSim();
    }

    private void setControlMotionMagic(double newTgt) {
        leftMotor.setControl(motionMagicRequest.withPosition(Math.min(Math.max(targetHeightMeters - ElevatorConstants.MIN_HEIGHT_GND_REL, ElevatorConstants.MIN_HEIGHT_ROBOT_REL), ElevatorConstants.MAX_HEIGHT_ROBOT_REL)));
        rightMotor.setControl(followRequest);
    }

    private double getMotionMagicReqTgt() {
        return motionMagicRequest.Position + ElevatorConstants.MIN_HEIGHT_GND_REL;
    }


    private void limitSwichCheck() {
        boolean fwd = getForwardLimitTriggered();
        boolean rev = getReverseLimitTriggered();
        if (rev && !revLimitTriggeredPrevCycle) {
            leftMotor.setRotorPosition(ElevatorConstants.MIN_ROT);
        } else if (fwd && !fwdLimitTriggeredPrevCycle) {
            leftMotor.setRotorPosition(ElevatorConstants.MAX_ROT);
        }

        if (fwd || rev) {
            hasBeenCalibrated = true;
        }

        revLimitTriggeredPrevCycle = rev;
        fwdLimitTriggeredPrevCycle = fwd;
    }

    private void calibrationRoutineCheck() {
        if (DriverStation.isEnabled() && !bypassCalibrationRoutine) {
            if (!hasBeenCalibrated && currentState != ElevatorControlMode.MANUAL) {
                calibrate();
            }
        } else {
            if (!bypassCalibrationRoutine && currentState != ElevatorControlMode.CALIBRATING) {
                if (hasBeenCalibrated) {
                    setLocked();
                } else {
                    setNeutral();
                }
            }
        } 
    }

    private void performSaftyControlResets() {
        if (DriverStation.isDisabled() || currentState != ElevatorControlMode.AUTOMATIC) {
            targetHeightMeters = getHeight();
        }

        if (DriverStation.isDisabled() || currentState != ElevatorControlMode.MANUAL) {
            manualControlDutyCycle = 0.0;
        }
    }

    private void performLimitAndCalibrationCheck() {
        if (RobotBase.isReal()) {
            limitSwichCheck();
            calibrationRoutineCheck();
        } 
    }

    @Override
    public void periodic() {

        //Elevator calibrates by hitting limit switch and resetting
        performLimitAndCalibrationCheck();
        
        performSaftyControlResets();
        

        if (!isForceStoped) {
            switch (currentState) { 
                case AUTOMATIC:
                    if (targetHeightMeters != getMotionMagicReqTgt()) {
                        setControlMotionMagic(targetHeightMeters);
                    }
                    break;
                case MANUAL:
                    leftMotor.setControl(dutyCycleRequest.withOutput(manualControlDutyCycle));
                    break;
                case CALIBRATING:
                    if (RobotBase.isReal() || !bypassCalibrationRoutine) {
                        if (!calibrationRoutine.isScheduled() && DriverStation.isEnabled()) {
                            calibrationRoutine.schedule();
                        }
                    } else {
                        currentState = ElevatorControlMode.AUTOMATIC;
                            hasBeenCalibrated = true;
                            BreakerLog.getInstance().logSuperstructureEvent("Elevator calibation not supported in sim, action fallthrough");
                    }
                    break;
                case NEUTRAL:
                    leftMotor.setControl(neutralRequest);
                    break;
                case LOCKED:
                default:
                    leftMotor.setControl(lockRequest);
                    break;
            }
        } else {
            leftMotor.setControl(lockRequest);
            rightMotor.setControl(lockRequest);
        }
    }

    private class CalibrationRoutine extends CommandBase {
        /** Creates a new CalibrationRoutine. */
        private final Timer timer = new Timer();
        private boolean bottomCalibration;
        public CalibrationRoutine() {}
      
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            bottomCalibration = true;
            timer.restart();
        }
      
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {

            if (DriverStation.isDisabled()) {
                this.cancel();
            }

            if (bottomCalibration) {
                leftMotor.setControl(dutyCycleRequest.withOutput(ElevatorConstants.BOTTOM_CALIBRATION_DUTY_CYCLE));
                if (timer.hasElapsed(ElevatorConstants.CALIBRATION_PAHSE_TIMEOUT)) {
                    bottomCalibration = false;
                }
            } else {
                leftMotor.setControl(dutyCycleRequest.withOutput(ElevatorConstants.BOTTOM_CALIBRATION_DUTY_CYCLE));
                if (timer.hasElapsed(ElevatorConstants.CALIBRATION_PAHSE_TIMEOUT * 2)) {
                    this.cancel();
                }
            }  
        }
      
        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                currentState = ElevatorControlMode.LOCKED;
                hasBeenCalibrated = false;
                BreakerLog.getInstance().logSuperstructureEvent(String.format("Elevator zero-point calibration FAILED (timeout or robot disable) (Time: %.2f)", timer.get()));
            } else {
                currentState = ElevatorControlMode.AUTOMATIC;
                hasBeenCalibrated = true;
            }
            timer.stop();
        }
      
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            boolean topLim = getForwardLimitTriggered();
            boolean botLim = getReverseLimitTriggered();
            if (topLim) {
                BreakerLog.getInstance().logSuperstructureEvent(String.format("Elevator zero-point calibration sucessfull (FWD) (Time: %.2f)", timer.get()));
            }
            if (botLim) {
                BreakerLog.getInstance().logSuperstructureEvent(String.format("Elevator zero-point calibration sucessfull (REV) (Time: %.2f)", timer.get()));
            }
          return topLim || botLim;
        }
      }

    public static enum ElevatorControlMode {
        CALIBRATING,
        AUTOMATIC,
        MANUAL,
        LOCKED,
        NEUTRAL
    }

    public static enum ElevatorTargetState {
        PLACE_HYBRID(0.0),
        PLACE_CONE_MID(0.0),
        PLACE_CONE_HIGH(0.0),
        PLACE_CUBE_MID(0.0),
        PLACE_CUBE_HIGH(0.0),
        PICKUP_GROUND_CONE(0.0),
        PICKUP_GROUND_CUBE(0.0),
        PICKUP_SINGLE_SUBSTATION_CONE(0.0),
        PICKUP_SINGLE_SUBSTATION_CUBE(0.0),
        PICKUP_DOUBLE_SUBSTATION_CONE(0.0),
        PICKUP_DOUBLE_SUBSTATION_CUBE(0.0),
        ARB_TEST_HEIGHT(0.6),
        STOW(ElevatorConstants.MIN_HEIGHT_GND_REL);

        private final double targetHeight;
        private ElevatorTargetState(double targetHeight) {
            this.targetHeight = targetHeight;
        }

        public double getTargetHeight() {
            return targetHeight;
        }

        public static Optional<ElevatorTargetState> getTargetFromHeight(double searchHeight) {
            for (ElevatorTargetState tgt: ElevatorTargetState.values()) {
                if (tgt.getTargetHeight() == searchHeight) {
                    return Optional.of(tgt);
                }
            }
            return Optional.empty();
        }
    }

    @Override
    public void toLog(LogTable table) {
        table.put("DeviceHealth", diagnostics.getHealth().toString());
        table.put("ControlMode", getCurrentControlMode().toString());
        table.put("TargetState", getTargetStateString());
        table.put("TargetHeightMeters", getTargetHeightMeters());
        table.put("HeightMeters", getHeight());
        table.put("VelocityMetersPerSec", getVelocity());
        table.put("ManualControlDutyCycle", manualControlDutyCycle);
        table.put("RawEncoderPos", leftMotor.getRotorPosition().getValue());
        table.put("HasBeenCalibrated", hasBeenCalibrated);
        table.put("IsForceStoped", isForceStoped());
        table.put("MotorMotionMagicRunning", leftMotor.getMotionMagicIsRunning().getValue().toString());
        table.put("ClosedLoopOut", leftMotor.getClosedLoopOutput().getValue());
        table.put("HighLimitTriggered", getForwardLimitTriggered());
        table.put("LowLimitTriggered", getReverseLimitTriggered());
    }


}
