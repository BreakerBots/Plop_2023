package frc.robot.BreakerLib.util.logging.advantagekit.console;

public interface ConsoleSource extends AutoCloseable {
    /**
     * Reads all console data that has been produced since the last call to this
     * method.
     */
    public String getNewData();
}
