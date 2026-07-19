// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.LEDConstants.Animations;
import frc.robot.utilities.*;
import frc.robot.utilities.DataLogUtil.RotationWhenDisabled;


public class LED extends SubsystemBase {
  private final int logRotationKey;
  private final String subsystemName;

  private final CANdle candle;
  private final LEDConfigs ledConfig = new LEDConfigs();

  // IMPORTANT: create an EmptyAnimation object for each slot that has an animation
  private final EmptyAnimation emptyAnimation0 = new EmptyAnimation(0);
  private final EmptyAnimation emptyAnimation1 = new EmptyAnimation(1);
  private final EmptyAnimation emptyAnimation2 = new EmptyAnimation(2);
  private final EmptyAnimation emptyAnimation3 = new EmptyAnimation(3);

  public static AnimationColor dashboardColor = AnimationColor.NEUTRAL;
  private boolean lastStickyFaultPresentReading = false;

  /**
   * Creates the LED subsystem.
   * @param CANPort the CAN port that the CANdle is on
   * @param CANBus the CANBus that the CANdle is on
   * @param subsystemName the name of the subsystem
   */
  public LED(int CANPort, CANBus CANBus, String subsystemName) {
    this.subsystemName = subsystemName;
    this.logRotationKey = DataLogUtil.allocateLogRotation();

    this.candle = new CANdle(CANPort, CANBus);

    // Create and apply config(s)
    ledConfig.withBrightnessScalar(0.25);
    candle.getConfigurator().apply(ledConfig);
  }
  
  /**
   * Gets the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Apply any animation to the LEDs and only clear slot specified in parameters.
   * @param animation animation object (ex: SolidColor, StrobeAnimation, ColorFlowAnimation, etc.)
   */
  public void animate(ControlRequest animation) {
    candle.setControl(emptyAnimation0);
    candle.setControl(animation);
  }

  /**
   * Clear all animations from the strips.
   */
  public void clearAllStripAnimations() {
    // add an empty animation for every slot in use!
    candle.setControl(emptyAnimation0);
    candle.setControl(emptyAnimation1);
    candle.setControl(emptyAnimation2);
    candle.setControl(emptyAnimation3);
  }

  @Override
  public void periodic() {
    if(DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kNormal)) {
      // If there wasn't a sticky fault last loop but there is one now, 
      // send sticky fault present event and update last reading
      if (!lastStickyFaultPresentReading && RobotPreferences.isStickyFaultActive()) {
        candle.setControl(Animations.CANdleStickyFaultAnimation);
        lastStickyFaultPresentReading = true;
      }

      // If there was a sticky fault last loop but there isn't one now,  
      // send sticky fault cleared event and update last reading
      else if (lastStickyFaultPresentReading && !RobotPreferences.isStickyFaultActive()) {
        candle.setControl(Animations.CANdleNeutralAnimation);
        lastStickyFaultPresentReading = false;
      }

      // update LED state and match timer on the dashboard
      SmartDashboard.putString("LED State", String.format("#%02x%02x%02x", dashboardColor.r, dashboardColor.g, dashboardColor.b));
    }
  }
}