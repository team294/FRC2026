// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LED;

public class LEDAnimateRobotDisabled extends Command {
  private LED led;
  private int cycleCounter;

  // amount of loops before the color changes, ex: 100 loops 20ms each 
  // is 2000ms which is 2 seconds, matching the duration of each animations
  private final int loopOffset = 100;

  /**
   * Creates an animation that pulses blue -> orange. Note that the orange pulsing
   * will continue forever if it is not cleared or replaced by another animation.
   * @param led LED subsystem
   */
  public LEDAnimateRobotDisabled(LED led) {
    this.led = led;

    addRequirements(led);
  }

  @Override
  public void initialize() {
    cycleCounter = 0;
  }

  @Override
  public void execute() {
    if (cycleCounter == 0) {
      led.animate(LEDConstants.Animations.disabledBlueAnimation);
    } else if (cycleCounter == loopOffset) {
      led.animate(LEDConstants.Animations.disabledOrangeAnimation);
    }

    cycleCounter++;
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    // run the second color for the same duration as the first
    return cycleCounter >= loopOffset * 2;
  }

  // Returns true if the command should run when the robot is disabled.
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
