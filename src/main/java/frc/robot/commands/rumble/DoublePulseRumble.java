// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.rumble;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.BreakerLib.driverstation.gamepad.BreakerGamepadTimedRumbleCommand;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoublePulseRumble extends SequentialCommandGroup {
  /** Creates a new DoublePulseRumble. */
  public DoublePulseRumble(BreakerXboxController driverControler, double leftStrength, double rightStrength, double pulseDuration, double pauseDuration) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BreakerGamepadTimedRumbleCommand(driverControler, pulseDuration, leftStrength, rightStrength),
      new WaitCommand(pauseDuration),
      new BreakerGamepadTimedRumbleCommand(driverControler, pulseDuration, leftStrength, rightStrength)
    );
  }

  public DoublePulseRumble(BreakerXboxController driverControler) {
    this(driverControler, 0.5, 0.5, 0.5, 0.5);
  }
}
