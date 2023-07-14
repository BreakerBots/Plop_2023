// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.devices.motors.smart;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import frc.robot.BreakerLib.devices.BreakerGenericDevice;
// import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestableBase;

// /** Add your docs here. */
// public abstract class BreakerSmartMotorController extends BreakerSelfTestableBase implements MotorController, AutoCloseable {

//     public final DutyCycleOutRequest setterRequest = new DutyCycleOutRequest(0.0, 0.0);
//     public abstract void setControl(BreakerSmartMotorControlRequest controlRequest); 

//     @Override
//     public void set(double speed) {
//         setControl(setterRequest.withDemand(speed));
//     }

//     public static class VoltageOutRequest extends BreakerSmartMotorControlRequest {
//         public VoltageOutRequest(double demand, double feedforward) {
//             super(BreakerSmartMotorControlType.OUTPUT, BreakerSmartMotorControlUnits.VOLTAGE, demand, feedforward);
//         }
//     }

//     public static class DutyCycleOutRequest extends BreakerSmartMotorControlRequest {
//         public DutyCycleOutRequest(double demand, double feedforward) {
//             super(BreakerSmartMotorControlType.OUTPUT, BreakerSmartMotorControlUnits.DUTY_CYCLE, demand, feedforward);
//         }
//     }

//     public static class NeutralRequest extends BreakerSmartMotorControlRequest {
//         public NeutralRequest() {
//             super(BreakerSmartMotorControlType.NEUTRAL, BreakerSmartMotorControlUnits.NEUTRAL, 0.0, 0.0);
//         }
//     }

//     protected static class BreakerSmartMotorControlRequest {
//         private BreakerSmartMotorControlType controlType;
//         private BreakerSmartMotorControlUnits controlUnits;
//         private double demand, feedforward;
//         public BreakerSmartMotorControlRequest(BreakerSmartMotorControlType controlType, BreakerSmartMotorControlUnits controlUnits, double demand, double feedforward) {
            
//         }

//         public BreakerSmartMotorControlType getControlType() {
//             return controlType;
//         }

//         public BreakerSmartMotorControlUnits getControlUnits() {
//             return controlUnits;
//         }

//         public double getDemand() {
//             return demand;
//         }

//         public double getFeedforward() {
//             return feedforward;
//         }

//         public BreakerSmartMotorControlRequest withDemand(double demand) {
//             this.demand = demand;
//             return this;
//         }


//         public BreakerSmartMotorControlRequest withFeedforward(double feedforward) {
//             this.feedforward = feedforward;
//             return this;
//         }

//     }

//     public static enum BreakerSmartMotorControlUnits {
//         VOLTAGE,
//         DUTY_CYCLE,
//         NEUTRAL
//     }

//     public static enum BreakerSmartMotorControlType {
//         OUTPUT,
//         SMART_MOTION,
//         POSITION,
//         VELOCITY,
//         NEUTRAL
//     }
// }
