// package frc.robot.BreakerLib.util.logging.advantagekit.inputs;

// import org.littletonrobotics.conduit.ConduitApi;
// import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
// import frc.robot.BreakerLib.util.logging.advantagekit.Logger;
// import frc.robot.BreakerLib.util.power.BreakerPowerManager;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.hal.PowerDistributionJNI;
// import edu.wpi.first.hal.simulation.PowerDistributionDataJNI;

// /**
//  * Manages logging power distribution data.
//  */
// public class LoggedPowerDistribution {

//   private static LoggedPowerDistribution instance;
//   private static final Logger logger = Logger.getInstance();

//   private final PowerDistributionInputs pdpInputs = new PowerDistributionInputs();

//   private int moduleID;
//   private int moduleType;

//   private LoggedPowerDistribution(int moduleID, PowerDistribution.ModuleType moduleType) {
//     this.moduleID = moduleID;
//     this.moduleType = moduleType.value;
//   }

//   private LoggedPowerDistribution() {
//     moduleID = PowerDistributionJNI.DEFAULT_MODULE;
//     moduleType = PowerDistributionJNI.AUTOMATIC_TYPE;
//     ConduitApi.getInstance().configurePowerDistribution(moduleID, this.moduleType);
//   }

//   public static LoggedPowerDistribution getInstance() {
//     if (instance == null) {
//       instance = new LoggedPowerDistribution();
//     }

//     return instance;
//   }

//   public static LoggedPowerDistribution getInstance(int moduleID, PowerDistribution.ModuleType moduleType) {
//     if (instance == null) {
//       instance = new LoggedPowerDistribution(moduleID, moduleType);
//     } else if (instance.moduleID != moduleID || instance.moduleType != moduleType.value) {
//       instance = new LoggedPowerDistribution(moduleID, moduleType);
//     }

//     return instance;
//   }

//   public static class PowerDistributionInputs implements LoggableInputs {
//     public double pdpTemperature;
//     public double pdpVoltage;
//     public double[] pdpChannelCurrents = new double[24];
//     public double pdpTotalCurrent;
//     public double pdpTotalPower;
//     public double pdpTotalEnergy;

//     public int channelCount;
//     public int handle;
//     public int type;
//     public int moduleId;
//     public long faults;
//     public long stickyFaults;

//     @Override
//     public void toLog(LogTable table) {
//       table.put("Temperature", pdpTemperature);
//       table.put("Voltage", pdpVoltage);
//       table.put("ChannelCurrent", pdpChannelCurrents);
//       table.put("TotalCurrent", pdpTotalCurrent);
//       table.put("TotalPower", pdpTotalPower);
//       table.put("TotalEnergy", pdpTotalEnergy);

//       table.put("ChannelCount", channelCount);
//       table.put("Faults", (int) faults);
//       table.put("StickyFaults", (int) stickyFaults);
//     }
//   }

//   public void periodic() {
//     // Update inputs from conduit
//       ConduitApi conduit = ConduitApi.getInstance();
//       pdpInputs.pdpTemperature = conduit.getPDPTemperature();
//       pdpInputs.pdpVoltage = conduit.getPDPVoltage();
//       for (int i = 0; i < 24; i++) {
//         pdpInputs.pdpChannelCurrents[i] = conduit.getPDPChannelCurrent(i);
//       }
//       pdpInputs.pdpTotalCurrent = conduit.getPDPTotalCurrent();
//       pdpInputs.pdpTotalPower = conduit.getPDPTotalPower();
//       pdpInputs.pdpTotalEnergy = conduit.getPDPTotalEnergy();
//       pdpInputs.channelCount = conduit.getPDPChannelCount();
//       pdpInputs.handle = conduit.getPDPHandle();
//       pdpInputs.type = conduit.getPDPType();
//       pdpInputs.moduleId = conduit.getPDPModuleId();
//       pdpInputs.faults = conduit.getPDPFaults();
//       pdpInputs.stickyFaults = conduit.getPDPStickyFaults();
//     }

//     logger.processInputs("PowerDistribution", pdpInputs);
//   }

//   public PowerDistributionInputs getInputs() {
//     return pdpInputs;
//   }

// }
