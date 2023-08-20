// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Drake;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GamePieceType;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;
import frc.robot.BreakerLib.driverstation.dashboard.BreakerDashboard;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveCANcoder;
import frc.robot.BreakerLib.util.BreakerRoboRIO;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.selftest.SystemDiagnostics;
import frc.robot.Constants.HandConstants;
import frc.robot.subsystems.Hand.RollerState.RollerStateType;
import frc.robot.subsystems.Hand.WristGoal.WristGoalType;

public class Hand extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax wristMotor;
  private final CANSparkMax rollerMotor;
  private final BreakerBeamBreak coneBeamBrake;
  private final BreakerBeamBreak cubeBeamBrake;

  private final SparkMaxLimitSwitch extendLimitSwitch;
  private final SparkMaxLimitSwitch retractLimitSwich;

  private ProfiledPIDController pid;
  private ArmFeedforward ff;

  private RollerState rollerState;
  private WristControlState wristControlState;
  private Rotation2d wristGoal;
  private WristGoalType wristGoalType;
  private CANcoder encoder;

  private final SystemDiagnostics diagnostics;
  public Hand() {
    wristMotor = new CANSparkMax(HandConstants.WRIST_ID, MotorType.kBrushless);
    rollerMotor = new CANSparkMax(HandConstants.ROLLER_ID, MotorType.kBrushless);
    coneBeamBrake = new BreakerBeamBreak(HandConstants.CONE_BEAM_BRAKE_DIO_PORT, HandConstants.BEAM_BRAKE_BROKEN_ON_TRUE);
    cubeBeamBrake = new BreakerBeamBreak(HandConstants.CUBE_BEAM_BRAKE_DIO_PORT, HandConstants.BEAM_BRAKE_BROKEN_ON_TRUE);
    
    wristMotor.setSmartCurrentLimit(HandConstants.WRIST_CURRENT_LIMIT);
    wristMotor.setInverted(HandConstants.INVERT_WRIST);
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.enableVoltageCompensation(12.0);
    

    rollerMotor.setInverted(HandConstants.INVERT_ROLLER);
    rollerMotor.setIdleMode(IdleMode.kBrake);
    rollerMotor.enableVoltageCompensation(12.0);

    extendLimitSwitch = wristMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    extendLimitSwitch.enableLimitSwitch(true);
    retractLimitSwich = wristMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    retractLimitSwich.enableLimitSwitch(true);

    pid = new ProfiledPIDController(HandConstants.WRIST_KP, HandConstants.WRIST_KI, HandConstants.WRIST_KD, new Constraints(HandConstants.WRIST_MAX_VELOCITY_RADS, HandConstants.WRIST_MAX_ACCELERATION_RADS_PER_SEC));
    ff = new ArmFeedforward(HandConstants.WRIST_KS, HandConstants.WRIST_KG, HandConstants.WRIST_KV, HandConstants.WRIST_KA);
    rollerState = RollerState.NEUTRAL;
    wristControlState = WristControlState.SEEKING;

    diagnostics = new SystemDiagnostics("Intake");
    diagnostics.addSparkMaxs(wristMotor, rollerMotor);
    diagnostics.addSupplier(this::healthCheck);

    wristMotor.burnFlash();
    rollerMotor.burnFlash();
    BreakerDashboard.getMainTab().add("INTAKE", this);

    wristGoal = getWristRotation();
  }

  private void setRollerMotor(double dutyCycle, int currentLimit) {
    rollerMotor.set(dutyCycle);
    rollerMotor.setSmartCurrentLimit(currentLimit);
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

    if (extendLimitSwitch.isPressed() && retractLimitSwich.isPressed()) {
      str += " actuator_limit_switch_malfunction_bolth_switches_read_triggered ";
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
    pid.calculate(getWristRotation().getRadians(), wristGoal.getRotations());
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


  @Override
  public void periodic() {

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
      calculateAndApplyPIDF();
    } else {
      privateSetWristGoal(WristGoalType.UNKNOWN, getWristRotation());
      stopRoller();
    }
    
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
    STOW(WristGoalType.STOW, Rotation2d.fromDegrees(0));

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



}
