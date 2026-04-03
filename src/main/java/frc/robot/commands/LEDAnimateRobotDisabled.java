// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LED;

public class LEDAnimateRobotDisabled extends Command {
  private final LED led;
  private final Timer timer;
  private boolean firstGroupSent;

  /**
   * Creates an animation that pulses blue -> orange. Note that the orange pulsing
   * will continue forever if it is not cleared or replaced by another animation.
   * @param led LED subsystem
   */
  public LEDAnimateRobotDisabled(LED led) {
    this.led = led;
    this.timer = new Timer();

    addRequirements(led);
  }

  @Override
  public void initialize() {
    firstGroupSent = false;
    timer.restart();
  }

  @Override
  public void execute() {
    if (!firstGroupSent && timer.get() >= 0.0 && timer.get() < 2.0) {
      led.animate(LEDConstants.Animations.disabledBlueAnimation1);
      led.animate(LEDConstants.Animations.disabledOrangeAnimation2);
      led.animate(LEDConstants.Animations.disabledBlueAnimation3);
      firstGroupSent = true;
    } else if (firstGroupSent && timer.get() >= 2.0) {
      led.animate(LEDConstants.Animations.disabledOrangeAnimation1);
      led.animate(LEDConstants.Animations.disabledBlueAnimation2);
      led.animate(LEDConstants.Animations.disabledOrangeAnimation3);
      firstGroupSent = false;
    }

    if (timer.get() >= 4.0) {
      timer.restart();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // Returns true if the command should run when the robot is disabled.
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
