// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.devices.motors.smart;

// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
// import com.ctre.phoenix6.hardware.TalonFX;

// import frc.robot.BreakerLib.devices.encoders.BreakerGenericEncoder;
// import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
// import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

// /** Add your docs here. */
// public class BreakerTalonFX extends TalonFX implements BreakerGenericSmartMotorController {

//     public BreakerTalonFX(int deviceId, String canbus) {
//         super(deviceId, canbus);
//         //TODO Auto-generated constructor stub
//     } 

//     public BreakerTalonFX(int deviceId) {
//         super(deviceId);
//     }

//     @Override
//     public void set(double speed) {
//         // TODO Auto-generated method stub
//         super.set(speed);
//     }

//     @Override
//     public void setVoltage(double volts) {
//         // TODO Auto-generated method stub
//         super.setVoltage(volts);
//     }

//     @Override
//     public void runSelfTest() {
//         // TODO Auto-generated method stub
        
//     }

//     @Override
//     public DeviceHealth getHealth() {
       
//         return null;
//     }

//     @Override
//     public String getFaults() {
//         // TODO Auto-generated method stub
//         return null;
//     }

//     @Override
//     public String getDeviceName() {
//         // TODO Auto-generated method stub
//         return null;
//     }

//     @Override
//     public boolean hasFault() {
//         // TODO Auto-generated method stub
//         return false;
//     }

//     @Override
//     public void setDeviceName(String newName) {
//         // TODO Auto-generated method stub
        
//     }

//     @Override
//     public void toLog(LogTable table) {
//         // TODO Auto-generated method stub
        
//     }

//     @Override
//     public void setBrakeMode() {
//         // TODO Auto-generated method stub
        
//     }

//     @Override
//     public double getDrawCurrent() {
//         // TODO Auto-generated method stub
//         return 0;
//     }

//     @Override
//     public BreakerGenericEncoder getEncoder(int encoderNum) {
//         switch (encoderNum) {
//             case 0:
//                 break;
        
//             default:
//                 break;
//         }
//         return null;
//     } 

//     public static class BreakerTalonFXRotorEncoder implements BreakerGenericEncoder {

//         private BreakerTalonFX parent;
//         public BreakerTalonFXRotorEncoder(BreakerTalonFX parent) {
//             this.parent = parent;
//         }

//         @Override
//         public void runSelfTest() {
//             // TODO Auto-generated method stub
            
//         }

//         @Override
//         public DeviceHealth getHealth() {
//             // TODO Auto-generated method stub
//             return null;
//         }

//         @Override
//         public String getFaults() {
//             // TODO Auto-generated method stub
//             return null;
//         }

//         @Override
//         public String getDeviceName() {
//             // TODO Auto-generated method stub
//             return null;
//         }

//         @Override
//         public boolean hasFault() {
//             // TODO Auto-generated method stub
//             return false;
//         }

//         @Override
//         public void setDeviceName(String newName) {
//             // TODO Auto-generated method stub
            
//         }

//         @Override
//         public double getEncoderAbsolutePosition() {
//             return parent.getConfigurator().apply(new FeedbackConfigs().FeedbackSensorSource);
//         }

//         @Override
//         public double getEncoderRelativePosition() {
//             // TODO Auto-generated method stub
//             return 0;
//         }

//         @Override
//         public double getEncoderVelocity() {
//             // TODO Auto-generated method stub
//             return 0;
//         }

//         @Override
//         public double setEncoderPosition(double newPosition) {
//             // TODO Auto-generated method stub
//             return 0;
//         }
        
//     }

    

// }
