package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants.AnimationColor;
import frc.robot.Constants.LEDConstants.*;
import frc.robot.subsystems.LED;


public class LEDEventUtil {
  private static LED led;
  private static RepeatCommand repeatedLEDAnimateRobotDisabled;

  private static LEDStripEvents previousEventStrip;

  public enum LEDStripEvents {
    INTAKING,
    REVERSE_INTAKING,
    PREPPING_SHOOTER,
    SHOOTER_READY,
    FEEDING_SHOOTER,
    LOWERING,
    TRENCH_SAFE,
    NEUTRAL,
    ROBOT_DISABLED
  }

  private static final Map<LEDStripEvents, Integer> prioritiesLEDStripEvents = new HashMap<>();
  
  static {
    prioritiesLEDStripEvents.put(LEDStripEvents.INTAKING, 1);
    prioritiesLEDStripEvents.put(LEDStripEvents.REVERSE_INTAKING, 1);
    prioritiesLEDStripEvents.put(LEDStripEvents.PREPPING_SHOOTER, 2);
    prioritiesLEDStripEvents.put(LEDStripEvents.SHOOTER_READY, 3);
    prioritiesLEDStripEvents.put(LEDStripEvents.FEEDING_SHOOTER, 4);
    prioritiesLEDStripEvents.put(LEDStripEvents.LOWERING, 5);
    prioritiesLEDStripEvents.put(LEDStripEvents.TRENCH_SAFE, 6);
    prioritiesLEDStripEvents.put(LEDStripEvents.NEUTRAL, 7);
    prioritiesLEDStripEvents.put(LEDStripEvents.ROBOT_DISABLED, 8);
  }

  /**
   * Sends an event to the LED Strips and update the LEDs if necessary.
   * @param event StripEvent event happening
   */
  public static void sendEvent(LEDStripEvents event) {
    // Always update state if previous event was neutral or disabled.
    // Do not update if last event was not neutral or disabled and the new event priority is less than the previous.

    if (led == null) {
      DataLogUtil.writeMessage("LED Strip SendEvent: LED is not instantiated (null)");
      return;
    }

    // if previous was neutral or disabled, always update.
    // if previous was lowering or displaying trench safety, always update.
    // otherwise, if priority is lower than current state, do not update (ex: intaking while feeding shooter should display the latter)
    if (
      previousEventStrip != LEDStripEvents.NEUTRAL && previousEventStrip != LEDStripEvents.ROBOT_DISABLED
      && previousEventStrip != LEDStripEvents.LOWERING && previousEventStrip != LEDStripEvents.TRENCH_SAFE 
      && getPriority(event) < getPriority(previousEventStrip)
    ) {
      DataLogUtil.writeMessage("LED Strip SendEvent: Event Priority Too Low");
      return;
    }

    switch (event) {
      case INTAKING:
        LED.dashboardColor = AnimationColor.INTAKING;
        led.animate(Animations.neutralAnimation);
        DataLogUtil.writeMessage("LED Strip Intaking");
        break;
      case REVERSE_INTAKING:
        LED.dashboardColor = AnimationColor.REVERSE_INTAKING;
        led.animate(Animations.reverseIntakingAnimation);
        DataLogUtil.writeMessage("LED Strip Reverse Intaking");
        break;
      case PREPPING_SHOOTER:
        LED.dashboardColor = AnimationColor.PREP_SHOOTER;
        led.animate(Animations.shooterPrepAnimation);
        DataLogUtil.writeMessage("LED Strip Prep Shooter");
        break;
      case SHOOTER_READY:
        LED.dashboardColor = AnimationColor.SHOOTER_READY;
        led.animate(Animations.shooterReadySolidColor);
        DataLogUtil.writeMessage("LED Strip Shooter Ready");
        break;
      case FEEDING_SHOOTER:
        LED.dashboardColor = AnimationColor.FEEDING_SHOOTER;
        led.animate(Animations.feedingShooterAnimation);
        DataLogUtil.writeMessage("LED Strip Feeding Shooter");
        break;
      case LOWERING:
        LED.dashboardColor = AnimationColor.LOWERING;
        led.animate(Animations.loweringAnimation);
        DataLogUtil.writeMessage("LED Strip Lowering Climber/Hood");
        break;
      case TRENCH_SAFE:
        LED.dashboardColor = AnimationColor.TRENCH_SAFE;
        led.animate(Animations.trenchSafeSolidColor);
        DataLogUtil.writeMessage("LED Strip Trench Safe");
        break;
      case ROBOT_DISABLED:
        LED.dashboardColor = AnimationColor.NEUTRAL;
        CommandScheduler.getInstance().schedule(repeatedLEDAnimateRobotDisabled);
        DataLogUtil.writeMessage("LED Strip Robot Disabled");
        break;
      default:
        LED.dashboardColor = AnimationColor.NEUTRAL;
        led.animate(Animations.neutralSolidColor);
        DataLogUtil.writeMessage("LED Strip Neutral");
        break;
    }

    // Update previous event since the LEDs were updated
    previousEventStrip = event;
  }

  /**
   * This method must be called once in RobotContainer
   * @param ledInstance LED subsystem
   * @param disabledAnimation disabled animation to play on strips (repeated!)
   */
  public static void initialize(LED ledInstance, RepeatCommand disabledAnimation) {
    led = ledInstance;
    repeatedLEDAnimateRobotDisabled = disabledAnimation;
  }

  /**
   * Gets the priority level for an event.
   * @param event LEDStripEvents event
   * @return priority level integer (higher value = higher priority), default is -1
   */
  private static int getPriority(LEDStripEvents event) {
    return event != null ? prioritiesLEDStripEvents.getOrDefault(event, -1) : -1;
  }
}
