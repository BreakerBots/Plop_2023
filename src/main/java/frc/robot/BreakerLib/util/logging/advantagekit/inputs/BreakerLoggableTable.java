// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.logging.advantagekit.inputs;

import java.util.Map;
import java.util.Map.Entry;

import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable.LogValue;

/** Add your docs here. */
public class BreakerLoggableTable implements LoggableInputs {
    Map<String, LogValue> data;
    public BreakerLoggableTable(String systemName) {

    }

    @Override
    public void toLog(LogTable table) {
        for (Entry<String, LogValue> ent: data.entrySet()) {
            switch (ent.getValue().type) {
                case Boolean:
                    break;
                case BooleanArray:
                    break;
                case Double:
                    break;
                case DoubleArray:
                    break;
                case Float:
                    break;
                case FloatArray:
                    break;
                case Integer:
                    break;
                case IntegerArray:
                    break;
                case Raw:
                    break;
                case String:
                    break;
                case StringArray:
                    break;
                default:
                    break;

            }
        }
        
    }

}
