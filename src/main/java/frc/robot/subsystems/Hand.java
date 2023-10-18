// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.counter.UpDownCounter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AllianceManager;
import frc.robot.GamePieceType;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;
import frc.robot.BreakerLib.devices.sensors.rangefinder.BreakerSEN36005;
import frc.robot.BreakerLib.driverstation.dashboard.BreakerDashboard;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLoggable;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.selftest.SystemDiagnostics;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;
import frc.robot.Constants.HandConstants;
import frc.robot.subsystems.Hand.RollerState.RollerStateType;
import frc.robot.subsystems.Hand.WristGoal.WristGoalType;

public class Hand extends SubsystemBase implements BreakerLoggable {
  /** Creates a new Intake. */
  private final TalonFX wristMotor;
  private final TalonFX rollerMotor;
  private final BreakerBeamBreak coneBeamBrake;
  private final BreakerBeamBreak cubeBeamBrake;

  private ProfiledPIDController pid;
  private ArmFeedforward ff;

  private RollerState rollerState;
  private WristControlState wristControlState;
  private Rotation2d wristGoal;
  private WristGoalType wristGoalType;

  private double pidOutput, ffOutput;

  private CANcoder encoder;
  private BreakerSEN36005 coneTOF;

  private ControledGamePieceType prevControledGamePieceType;

  private final SystemDiagnostics diagnostics;
  public Hand() {
    wristMotor = new TalonFX(HandConstants.WRIST_ID);
    rollerMotor = new TalonFX(HandConstants.ROLLER_ID);
    coneBeamBrake = new BreakerBeamBreak(HandConstants.CONE_BEAM_BRAKE_DIO_PORT, HandConstants.BEAM_BRAKE_BROKEN_ON_TRUE);
    cubeBeamBrake = new BreakerBeamBreak(HandConstants.CUBE_BEAM_BRAKE_DIO_PORT, HandConstants.BEAM_BRAKE_BROKEN_ON_TRUE);
    
    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    wristConfig.CurrentLimits.SupplyCurrentLimit = HandConstants.WRIST_CURRENT_LIMIT;
    wristConfig.CurrentLimits.SupplyTimeThreshold = HandConstants.WRIST_CURRENT_LIMIT_TIME;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.MotorOutput.Inverted = HandConstants.INVERT_ROLLER ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    MotorOutputConfigs rollerOutConfig = new MotorOutputConfigs();
    rollerOutConfig.Inverted = HandConstants.INVERT_ROLLER ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    rollerOutConfig.NeutralMode = NeutralModeValue.Brake;
    rollerMotor.getConfigurator().apply(rollerOutConfig);

    pid = new ProfiledPIDController(HandConstants.WRIST_KP, HandConstants.WRIST_KI, HandConstants.WRIST_KD, new Constraints(HandConstants.WRIST_MAX_VELOCITY_RADS, HandConstants.WRIST_MAX_ACCELERATION_RADS_PER_SEC));
    ff = new ArmFeedforward(HandConstants.WRIST_KS, HandConstants.WRIST_KG, HandConstants.WRIST_KV, HandConstants.WRIST_KA);
    rollerState = RollerState.NEUTRAL;
    wristControlState = WristControlState.SEEKING;

    encoder = BreakerCANCoderFactory.createCANCoder(HandConstants.WRIST_ENCODER_ID, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, -0.414551, SensorDirectionValue.Clockwise_Positive);

    diagnostics = new SystemDiagnostics("Intake");
    diagnostics.addPhoenix6TalonFXs(rollerMotor, wristMotor);
    diagnostics.addSupplier(this::healthCheck);

    BreakerDashboard.getMainTab().add("INTAKE", this);

    coneTOF = new BreakerSEN36005(7);
    coneTOF.setRangingMode(RangingMode.Short, 999);
    coneTOF.setRangeOfInterest(9,9,11,11);

    wristGoal = Rotation2d.fromDegrees(0.0); //getWristRotation();
    wristGoalType = WristGoalType.UNKNOWN;
    prevControledGamePieceType = ControledGamePieceType.NONE;
    BreakerLog.getInstance().registerLogable("Hand", this);
  }

  public Optional<Double> getConeOffset() {
    Optional<Alliance> ally = AllianceManager.getAlliance();
    double leftDist = coneTOF.getRange();
    boolean invalidDist = leftDist >= HandConstants.CONE_TOF_MAX_DIST || !coneTOF.isRangeValid();
    if (getControledGamePieceType() != ControledGamePieceType.CONE || invalidDist || ally.isEmpty()) {
      return Optional.empty();
    } else if (ally.get() == Alliance.Blue) {
      return Optional.of((coneTOF.getRange() - HandConstants.CENTERED_CONE_TOF_DISTANCE) / 1000.0);
    }
    return Optional.of((HandConstants.CENTERED_CONE_TOF_DISTANCE - coneTOF.getRange()) / 1000.0);
    // return Optional.of(pos + HandConstants.CONTROLED_CONE_AVG_RADIUS);
  }

  private void setRollerMotor(double dutyCycle, int currentLimit) {
    rollerMotor.set(dutyCycle);
    CurrentLimitsConfigs updatedCurLimits = new CurrentLimitsConfigs();
    rollerMotor.getConfigurator().refresh(updatedCurLimits);
    updatedCurLimits.SupplyCurrentLimit = currentLimit;
    updatedCurLimits.SupplyCurrentLimitEnable = true;
    rollerMotor.getConfigurator().apply(updatedCurLimits);

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("INTK HAS CONE", this::hasCone, null);
    builder.addBooleanProperty("INTK HAS CUBE", this::hasCone, null);
    builder.addBooleanProperty("INTK INTAKEING", () -> getRollerState().getRollerStateType() == RollerStateType.INTAKEING, null);
    builder.addBooleanProperty("INTK GRIPPING", () -> getRollerState().getRollerStateType() == RollerStateType.GRIPPING, null);
    builder.addBooleanProperty("INTK EXTAKEING", () -> getRollerState().getRollerStateType() == RollerStateType.EXTAKEING, null);
    // builder.addStringProperty("INTK A-MOT", () -> getActuatorMotorState().toString(), null);
    // builder.addStringProperty("INTK A-ST", () -> getActuatorState().toString(), null);
    builder.addStringProperty("INTK ROLL", () -> getRollerState().toString(), null);
    builder.addBooleanProperty("INTK FAULT", diagnostics::hasFault, null);
  }

  private Pair<DeviceHealth, String> healthCheck() {
    String str = "";
    DeviceHealth devHealth = DeviceHealth.NOMINAL;
    if (getControledGamePieceType() == ControledGamePieceType.ERROR) {
      str += " game_piece_deection_beam_break_malfunction_bolth_sensors_read_broken ";
      devHealth = DeviceHealth.INOPERABLE;
    }
    return new Pair<DeviceHealth,String>(devHealth, str);
  }

  public ControledGamePieceType getControledGamePieceType() {
    boolean cone = coneBeamBrake.isBroken();
    boolean cube = cubeBeamBrake.isBroken();
    if (cone && !cube) {
      return ControledGamePieceType.CONE;
    } else if (!cone && cube) {
      return ControledGamePieceType.CUBE;
    } else if (!cone && !cube) {
      return ControledGamePieceType.NONE;
    }
    return ControledGamePieceType.ERROR;
  }

  public boolean hasCone() {
    return getControledGamePieceType() == ControledGamePieceType.CONE;
  }

  public boolean hasCube() {
    return getControledGamePieceType() == ControledGamePieceType.CUBE;
  }

  public boolean hasGamePiece() {
    return getControledGamePieceType().hasGamePiece();
  }

  public WristState getWristState() {
    if (DriverStation.isDisabled() || wristControlState == WristControlState.IDLE) {
      return WristState.IDLE;
    } else if (atWristGoal()) {
      return WristState.AT_GOAL;
    }
    return WristState.TRANSIT;
  }

  public WristControlState getWristControlState() {
    return wristControlState;
  }

  public void rollerIntakeCone() {
    if (wristGoalType == WristGoalType.PICKUP && !hasGamePiece()) {
      rollerState = RollerState.INTAKEING_CONE;
      setRollerMotor(HandConstants.INTAKE_CONE_DUTY_CYCLE, HandConstants.INTAKE_CONE_CURENT_LIMIT);
    }
  }

  public void rollerIntakeCube() {
    if (wristGoalType == WristGoalType.PICKUP && !hasGamePiece()) {
      rollerState = RollerState.INTAKEING_CUBE;
      setRollerMotor(HandConstants.INTAKE_CUBE_DUTY_CYCLE, HandConstants.INTAKE_CUBE_CURENT_LIMIT);
    }
  }

  public void rollerExtakeCone() {
    if ((wristGoalType != WristGoalType.STOW || wristGoalType != WristGoalType.UNKNOWN) && getControledGamePieceType() == ControledGamePieceType.CONE) {
      rollerState = RollerState.EXTAKEING_CONE;
      setRollerMotor(HandConstants.EXTAKE_CONE_DUTY_CYCLE, HandConstants.EXTAKE_CONE_CURENT_LIMIT);
    }
  }

  public void rollerExtakeCube() {
    if ((wristGoalType != WristGoalType.STOW || wristGoalType != WristGoalType.UNKNOWN) && getControledGamePieceType() == ControledGamePieceType.CUBE) {
      rollerState = RollerState.EXTAKEING_CUBE;
      setRollerMotor(HandConstants.EXTAKE_CUBE_DUTY_CYCLE, HandConstants.EXTAKE_CUBE_CURENT_LIMIT);
    }
  }

  private void rollerGrippCone() {
    setRollerMotor(HandConstants.INTAKE_CONE_GRIP_DUTY_CYCLE, HandConstants.INTAKE_CONE_GRIP_CURENT_LIMIT);
    rollerState = RollerState.GRIPPING_CONE;
  }

  private void rollerGrippCube() {
    setRollerMotor(HandConstants.INTAKE_CUBE_GRIP_DUTY_CYCLE, HandConstants.INTAKE_CUBE_GRIP_CURENT_LIMIT);
    rollerState = RollerState.GRIPPING_CUBE;
  }

  public void stopRoller() {
    if (!hasGamePiece()) {
      rollerState = RollerState.NEUTRAL;
      rollerMotor.stopMotor();
    }
  }

  public RollerState getRollerState() {
      return rollerState;
  }

  public void setWristGoal(WristGoal wristGoal) {
    privateSetWristGoal(wristGoal.getGoalType(), wristGoal.getGoalAngle());
  }

  private void privateSetWristGoal(WristGoalType goalType, Rotation2d wristGoal) {
    this.wristGoal = wristGoal;
    this.wristGoalType = goalType;
    
  }
 
  private void calculateAndApplyPIDF() {
    pidOutput = pid.calculate(getWristRotation().getRadians(), wristGoal.getRadians());
    State profiledSetpoint = pid.getSetpoint();
    ffOutput =  ff.calculate(profiledSetpoint.position, profiledSetpoint.velocity);
    System.out.println(pidOutput + ffOutput);
    wristMotor.set((pidOutput + ffOutput)/RobotController.getBatteryVoltage());
  }

  public boolean atWristGoal() {
    return BreakerMath.epsilonEquals(wristGoal.getDegrees(), getWristRotation().getDegrees(), 0);
  }

  public Rotation2d getWristRotation() {
    return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue());
  }

  public WristGoalType getWristGoalType() {
      return wristGoalType;
  }

  public Rotation2d getWristGoalAngle() {
    return wristGoal;
  }


  @Override
  public void periodic() {

    //calculateAndApplyPIDF();

    if (DriverStation.isEnabled() && wristControlState == WristControlState.SEEKING) {
      if (hasGamePiece() && rollerState.getRollerStateType() != RollerStateType.EXTAKEING) {
        if (hasCone()) {
          rollerGrippCone();
        } else {
          rollerGrippCube();
        }
      } else if (!hasGamePiece() && (wristGoalType == WristGoalType.STOW || wristGoalType == WristGoalType.UNKNOWN)) {
        stopRoller();
      }
      //calculateAndApplyPIDF();
    } else {
      privateSetWristGoal(WristGoalType.UNKNOWN, getWristRotation());
      stopRoller();
    }

    // ControledGamePieceType curControledGamePieceType = getControledGamePieceType();
    // if (curControledGamePieceType != prevControledGamePieceType) {
    //   if (curControledGamePieceType == ControledGamePieceType.CONE || curControledGamePieceType == ControledGamePieceType.CUBE) {
    //     BreakerLog.getInstance().logSuperstructureEvent("ROBOT AQUIRED NEW GAME PIECE: " + curControledGamePieceType.toString());
    //   } else if (curControledGamePieceType == ControledGamePieceType.NONE && prevControledGamePieceType != ControledGamePieceType.ERROR) {
    //     BreakerLog.getInstance().logSuperstructureEvent("ROBOT EJECTED GAME PIECE: " + prevControledGamePieceType.toString());
    //   } else if (prevControledGamePieceType == ControledGamePieceType.ERROR) {
    //     BreakerLog.getInstance().logSuperstructureEvent("INTAKE BEAM BREAKS EXITED ERROR STATE, CURRENT GAME PIECE: " + curControledGamePieceType.toString());
    //   } else {
    //     BreakerLog.getInstance().logSuperstructureEvent("INTKAE BEAM BREAKS ENTERED ERROR STATE");
    //   }
    // }
  }

  public static enum RollerState {
    INTAKEING_CONE(RollerStateType.INTAKEING, ControledGamePieceType.CONE),
    INTAKEING_CUBE(RollerStateType.INTAKEING, ControledGamePieceType.CUBE),
    EXTAKEING_CONE(RollerStateType.EXTAKEING, ControledGamePieceType.CONE),
    EXTAKEING_CUBE(RollerStateType.EXTAKEING, ControledGamePieceType.CUBE),
    GRIPPING_CONE(RollerStateType.GRIPPING, ControledGamePieceType.CONE),
    GRIPPING_CUBE(RollerStateType.GRIPPING, ControledGamePieceType.CUBE),
    NEUTRAL(RollerStateType.NEUTRAL, ControledGamePieceType.NONE);

    private RollerStateType rollerStateType;
    private ControledGamePieceType desiredControledGamePieceType;
    private RollerState(RollerStateType rollerStateType, ControledGamePieceType desiredControledGamePieceType) {
      this.rollerStateType = rollerStateType;
      this.desiredControledGamePieceType = desiredControledGamePieceType;
    }

    public ControledGamePieceType getDesiredControledGamePieceType() {
        return desiredControledGamePieceType;
    }

    public RollerStateType getRollerStateType() {
        return rollerStateType;
    }

    public static enum RollerStateType {
      INTAKEING,
      EXTAKEING,
      GRIPPING,
      NEUTRAL
    }
    
  }

  public static enum ControledGamePieceType {
    CONE(Optional.of(GamePieceType.CONE)),
    CUBE(Optional.of(GamePieceType.CUBE)),
    NONE(Optional.empty()),
    ERROR(Optional.empty());


    private final Optional<GamePieceType> controledType;
    private ControledGamePieceType(Optional<GamePieceType> controledType) {
      this.controledType = controledType;
    }

    public Optional<GamePieceType> getGamePieceType() {
        return controledType;
    }

    public boolean hasGamePiece() {
      return this != ERROR && this != NONE;
    }


  }

  public static enum WristState {
    AT_GOAL,
    TRANSIT,
    IDLE
  }

  public static enum WristControlState {
    SEEKING,
    IDLE
  }

  public static enum WristGoal{
    PLACE_HYBRID(WristGoalType.PLACE, Rotation2d.fromDegrees(0)),
    PLACE_CONE_MID(WristGoalType.PLACE, Rotation2d.fromDegrees(0)),
    PLACE_CONE_HIGH(WristGoalType.PLACE, Rotation2d.fromDegrees(0)),
    PLACE_CUBE_MID(WristGoalType.PLACE, Rotation2d.fromDegrees(0)),
    PLACE_CUBE_HIGH(WristGoalType.PLACE, Rotation2d.fromDegrees(0)),
    PICKUP_GROUND_CONE(WristGoalType.PICKUP, Rotation2d.fromDegrees(0)),
    PICKUP_GROUND_CUBE(WristGoalType.PICKUP, Rotation2d.fromDegrees(0)),
    PICKUP_SINGLE_SUBSTATION_CONE(WristGoalType.PICKUP, Rotation2d.fromDegrees(0)),
    PICKUP_SINGLE_SUBSTATION_CUBE(WristGoalType.PICKUP, Rotation2d.fromDegrees(0)),
    PICKUP_DOUBLE_SUBSTATION_CONE(WristGoalType.PICKUP, Rotation2d.fromDegrees(0)),
    PICKUP_DOUBLE_SUBSTATION_CUBE(WristGoalType.PICKUP, Rotation2d.fromDegrees(0)),
    STOW(WristGoalType.STOW, Rotation2d.fromDegrees(90));

        private final Rotation2d goalAngle;
        private final WristGoalType goalType;
        private WristGoal(WristGoalType goalType, Rotation2d goalAngle) {
            this.goalAngle = goalAngle;
            this.goalType = goalType;
        }

        public Rotation2d getGoalAngle() {
            return goalAngle;
        }

        public WristGoalType getGoalType() {
            return goalType;
        }

      public static enum WristGoalType {
        PLACE,
        PICKUP,
        STOW,
        UNKNOWN
      }
  }

  @Override
  public void toLog(LogTable table) {
    table.put("DeviceHealth", diagnostics.getHealth().toString());
    table.put("ControlledGamePieceType", getControledGamePieceType().toString());
    table.put("ConeRangeRaw", coneTOF.getRange());
    table.put("ConeDistanceOffsestMeters", getConeOffset().toString());
    LogTable rollerTable = table.getSubtable("roller");
      rollerTable.put("State", rollerState.toString());
      rollerTable.put("DutyCycle", rollerMotor.get());
      rollerTable.put("CurrentAmps", rollerMotor.getSupplyCurrent().getValue());
    LogTable wristTable = table.getSubtable("Wrist");
      wristTable.put("ControlState", wristControlState.toString());
      wristTable.put("GoalType", wristGoalType.toString());
      wristTable.put("GoalDeg", wristGoal.getDegrees());
      wristTable.put("PositionDeg", getWristRotation().getDegrees());
      wristTable.put("FeedForwardVolts", ffOutput);
      wristTable.put("PIDVolts", pidOutput);
      wristTable.put("MotorOutVolts", wristMotor.getDutyCycle().getValue());
      wristTable.put("CurrentAmps", wristMotor.getSupplyCurrent().getValue());
  }



}
