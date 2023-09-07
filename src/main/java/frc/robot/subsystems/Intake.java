// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.Optional;
// import java.util.function.Supplier;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
// import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
// import com.ctre.phoenix6.signals.ForwardLimitValue;
// import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
// import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
// import com.ctre.phoenix6.signals.ReverseLimitValue;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxLimitSwitch;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.GamePieceType;
// import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;
// import frc.robot.BreakerLib.driverstation.dashboard.BreakerDashboard;
// import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
// import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
// import frc.robot.BreakerLib.util.test.selftest.SystemDiagnostics;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.subsystems.Intake.RollerState.RollerStateType;

// public class Intake extends SubsystemBase {
//   /** Creates a new Intake. */
//   private final CANSparkMax actuatorMotor;
//   private final CANSparkMax rollerMotor;
//   private final BreakerBeamBreak coneBeamBrake;
//   private final BreakerBeamBreak cubeBeamBrake;

//   private final SparkMaxLimitSwitch extendLimitSwitch;
//   private final SparkMaxLimitSwitch retractLimitSwich;

//   private RollerState rollerState;
//   private ActuatorMotorState actuatorMotorState;

//   private final SystemDiagnostics diagnostics;
//   public Intake() {
//     actuatorMotor = new CANSparkMax(IntakeConstants.ACTUATOR_ID, MotorType.kBrushless);
//     rollerMotor = new CANSparkMax(IntakeConstants.ROLLER_ID, MotorType.kBrushless);
//     coneBeamBrake = new BreakerBeamBreak(IntakeConstants.CONE_BEAM_BRAKE_DIO_PORT, IntakeConstants.BEAM_BRAKE_BROKEN_ON_TRUE);
//     cubeBeamBrake = new BreakerBeamBreak(IntakeConstants.CUBE_BEAM_BRAKE_DIO_PORT, IntakeConstants.BEAM_BRAKE_BROKEN_ON_TRUE);
    
//     actuatorMotor.setSmartCurrentLimit(IntakeConstants.ACTUATOR_CURRENT_LIMIT);
//     actuatorMotor.setInverted(IntakeConstants.INVERT_ACTUATOR);
//     actuatorMotor.setIdleMode(IdleMode.kBrake);
//     actuatorMotor.enableVoltageCompensation(12.0);
    

//     rollerMotor.setInverted(IntakeConstants.INVERT_ROLLER);
//     rollerMotor.setIdleMode(IdleMode.kBrake);
//     rollerMotor.enableVoltageCompensation(12.0);
    

//     extendLimitSwitch = actuatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
//     extendLimitSwitch.enableLimitSwitch(true);
//     retractLimitSwich = actuatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
//     retractLimitSwich.enableLimitSwitch(true);
    
//     rollerState = RollerState.NEUTRAL;
//     actuatorMotorState = getActuatorState() == ActuatorState.EXTENDED ? ActuatorMotorState.EXTENDING : ActuatorMotorState.RETRACTING;

//     diagnostics = new SystemDiagnostics("Intake");
//     diagnostics.addSparkMaxs(actuatorMotor, rollerMotor);
//     diagnostics.addSupplier(this::healthCheck);

//     actuatorMotor.burnFlash();
//     rollerMotor.burnFlash();
//     BreakerDashboard.getMainTab().add("INTAKE", this);
//   }

//   private void setRollerMotor(double dutyCycle, int currentLimit) {
//     rollerMotor.set(dutyCycle);
//     rollerMotor.setSmartCurrentLimit(currentLimit);
//   }

//   @Override
//   public void initSendable(SendableBuilder builder) {
//     builder.addBooleanProperty("INTK HAS CONE", this::hasCone, null);
//     builder.addBooleanProperty("INTK HAS CUBE", this::hasCone, null);
//     builder.addBooleanProperty("INTK INTAKEING", () -> getRollerState().getRollerStateType() == RollerStateType.INTAKEING, null);
//     builder.addBooleanProperty("INTK GRIPPING", () -> getRollerState().getRollerStateType() == RollerStateType.GRIPPING, null);
//     builder.addBooleanProperty("INTK EXTAKEING", () -> getRollerState().getRollerStateType() == RollerStateType.EXTAKEING, null);
//     builder.addStringProperty("INTK A-MOT", () -> getActuatorMotorState().toString(), null);
//     builder.addStringProperty("INTK A-ST", () -> getActuatorState().toString(), null);
//     builder.addStringProperty("INTK ROLL", () -> getRollerState().toString(), null);
//     builder.addBooleanProperty("INTK FAULT", diagnostics::hasFault, null);
//   }

//   private Pair<DeviceHealth, String> healthCheck() {
//     String str = "";
//     DeviceHealth devHealth = DeviceHealth.NOMINAL;
//     if (getControledGamePieceType() == ControledGamePieceType.ERROR) {
//       str += " game_piece_deection_beam_break_malfunction_bolth_sensors_read_broken ";
//       devHealth = DeviceHealth.INOPERABLE;
//     }

//     if (getActuatorState() == ActuatorState.ERROR) {
//       str += " actuator_limit_switch_malfunction_bolth_switches_read_triggered ";
//       devHealth = DeviceHealth.INOPERABLE;
//     }
//     return new Pair<DeviceHealth,String>(devHealth, str);
//   }

//   public ControledGamePieceType getControledGamePieceType() {
//     boolean cone = coneBeamBrake.isBroken();
//     boolean cube = cubeBeamBrake.isBroken();
//     if (cone && !cube) {
//       return ControledGamePieceType.CONE;
//     } else if (!cone && cube) {
//       return ControledGamePieceType.CUBE;
//     } else if (!cone && !cube) {
//       return ControledGamePieceType.NONE;
//     }
//     return ControledGamePieceType.ERROR;
//   }

//   public boolean hasCone() {
//     return getControledGamePieceType() == ControledGamePieceType.CONE;
//   }

//   public boolean hasCube() {
//     return getControledGamePieceType() == ControledGamePieceType.CUBE;
//   }

//   public boolean hasGamePiece() {
//     return getControledGamePieceType().hasGamePiece();
//   }

//   public ActuatorMotorState getActuatorMotorState() {
//     return actuatorMotorState;
//   }

//   public boolean isInDesiredActuatorState() {
//     return actuatorMotorState == ActuatorMotorState.EXTENDING ? getActuatorState() == ActuatorState.EXTENDED : getActuatorState() == ActuatorState.RETRACTED;
//   }

//   public ActuatorState getActuatorState() {
//     boolean extLimit = extendLimitSwitch.isPressed();
//     boolean retLimit = retractLimitSwich.isPressed();
//     if (extLimit && !retLimit) {
//       return ActuatorState.EXTENDED;
//     } else if (!extLimit && retLimit) {
//       return ActuatorState.RETRACTED;
//     } else if (!extLimit && !retLimit) {
//       return ActuatorState.TRANSIT;
//     } 
//     return ActuatorState.ERROR;
//   } 

//   public void intakeCone() {
//     if (actuatorMotorState == ActuatorMotorState.EXTENDING && !hasGamePiece()) {
//       rollerState = RollerState.INTAKEING_CONE;
//       setRollerMotor(IntakeConstants.INTAKE_CONE_DUTY_CYCLE, IntakeConstants.INTAKE_CONE_CURENT_LIMIT);
//     }
//   }

//   public void intakeCube() {
//     if (actuatorMotorState == ActuatorMotorState.EXTENDING && !hasGamePiece()) {
//       rollerState = RollerState.INTAKEING_CUBE;
//       setRollerMotor(IntakeConstants.INTAKE_CUBE_DUTY_CYCLE, IntakeConstants.INTAKE_CUBE_CURENT_LIMIT);
//     }
//   }

//   public void extakeCone() {
//     if (actuatorMotorState == ActuatorMotorState.EXTENDING && getControledGamePieceType() == ControledGamePieceType.CONE) {
//       rollerState = RollerState.EXTAKEING_CONE;
//       setRollerMotor(IntakeConstants.EXTAKE_CONE_DUTY_CYCLE, IntakeConstants.EXTAKE_CONE_CURENT_LIMIT);
//     }
//   }

//   public void extakeCube() {
//     if (actuatorMotorState == ActuatorMotorState.EXTENDING && getControledGamePieceType() == ControledGamePieceType.CUBE) {
//       rollerState = RollerState.EXTAKEING_CUBE;
//       setRollerMotor(IntakeConstants.EXTAKE_CUBE_DUTY_CYCLE, IntakeConstants.EXTAKE_CUBE_CURENT_LIMIT);
//     }
//   }

//   private void grippCone() {
//     setRollerMotor(IntakeConstants.INTAKE_CONE_GRIP_DUTY_CYCLE, IntakeConstants.INTAKE_CONE_GRIP_CURENT_LIMIT);
//     rollerState = RollerState.GRIPPING_CONE;
//   }

//   private void grippCube() {
//     setRollerMotor(IntakeConstants.INTAKE_CUBE_GRIP_DUTY_CYCLE, IntakeConstants.INTAKE_CUBE_GRIP_CURENT_LIMIT);
//     rollerState = RollerState.GRIPPING_CUBE;
//   }

//   public void stopRoller() {
//     if (!hasGamePiece()) {
//       rollerState = RollerState.NEUTRAL;
//       rollerMotor.stopMotor();
//     }
//   }

//   public RollerState getRollerState() {
//       return rollerState;
//   }

//   public void setActuatorMotorState(ActuatorMotorState newState) {
//     if (actuatorMotorState != newState) {
//       BreakerLog.getInstance().logSuperstructureEvent(String.format("INTAKE NOW %s", newState));
//     }
//     actuatorMotorState = newState;
//     privateSetActuatorMotorState(newState);
    
//   }

//   public void extend() {
//     setActuatorMotorState(ActuatorMotorState.EXTENDING);
//   }

//   public void retract() {
//     setActuatorMotorState(ActuatorMotorState.RETRACTING);
//   }

//   private void privateSetActuatorMotorState(ActuatorMotorState newState) {
//     switch(newState) {
//       case EXTENDING:
//         actuatorMotor.set(IntakeConstants.ACTUATOR_EXTEND_DUTY_CYCLE);
//         break;
//       case RETRACTING:
//         actuatorMotor.set(IntakeConstants.ACTUATOR_RETRACT_DUTY_CYCLE);
//         break;
//       default:
//         actuatorMotor.stopMotor();
//         break;
      
//     }
//   }

//   @Override
//   public void periodic() {

//     if (DriverStation.isEnabled()) {
//       if (hasGamePiece() && rollerState.getRollerStateType() != RollerStateType.EXTAKEING) {
//         if (hasCone()) {
//           grippCone();
//         } else {
//           grippCube();
//         }
//       } else if (!hasGamePiece() && actuatorMotorState != ActuatorMotorState.EXTENDING) {
//         stopRoller();
//       }

//       privateSetActuatorMotorState(actuatorMotorState);
//     } else {
//       stopRoller();
//     }
    
//   }

//   public static enum RollerState {
//     INTAKEING_CONE(RollerStateType.INTAKEING, ControledGamePieceType.CONE),
//     INTAKEING_CUBE(RollerStateType.INTAKEING, ControledGamePieceType.CUBE),
//     EXTAKEING_CONE(RollerStateType.EXTAKEING, ControledGamePieceType.CONE),
//     EXTAKEING_CUBE(RollerStateType.EXTAKEING, ControledGamePieceType.CUBE),
//     GRIPPING_CONE(RollerStateType.GRIPPING, ControledGamePieceType.CONE),
//     GRIPPING_CUBE(RollerStateType.GRIPPING, ControledGamePieceType.CUBE),
//     NEUTRAL(RollerStateType.NEUTRAL, ControledGamePieceType.NONE);

//     private RollerStateType rollerStateType;
//     private ControledGamePieceType desiredControledGamePieceType;
//     private RollerState(RollerStateType rollerStateType, ControledGamePieceType desiredControledGamePieceType) {
//       this.rollerStateType = rollerStateType;
//       this.desiredControledGamePieceType = desiredControledGamePieceType;
//     }

//     public ControledGamePieceType getDesiredControledGamePieceType() {
//         return desiredControledGamePieceType;
//     }

//     public RollerStateType getRollerStateType() {
//         return rollerStateType;
//     }

//     public static enum RollerStateType {
//       INTAKEING,
//       EXTAKEING,
//       GRIPPING,
//       NEUTRAL
//     }
    
//   }

//   public static enum ActuatorState {
//     EXTENDED,
//     RETRACTED,
//     TRANSIT,
//     ERROR
//   }

//   public static enum ControledGamePieceType {
//     CONE(Optional.of(GamePieceType.CONE)),
//     CUBE(Optional.of(GamePieceType.CUBE)),
//     NONE(Optional.empty()),
//     ERROR(Optional.empty());


//     private final Optional<GamePieceType> controledType;
//     private ControledGamePieceType(Optional<GamePieceType> controledType) {
//       this.controledType = controledType;
//     }

//     public Optional<GamePieceType> getGamePieceType() {
//         return controledType;
//     }

//     public boolean hasGamePiece() {
//       return this != ERROR && this != NONE;
//     }


//   }

//   public static enum ActuatorMotorState {
//     /** Motor is actively extending or is holding the intake in an extended state, regardless of limit switch state or if robot is enabled */
//     EXTENDING,
//     /** Motor is actively retracting or is holding the intake in an retracted state, regardless of limit switch state or if robot is enabled */
//     RETRACTING,
//     // /** Motor is not active and is not holding the intake in any particular state */
//     // NEUTRAL
//   }



// }
