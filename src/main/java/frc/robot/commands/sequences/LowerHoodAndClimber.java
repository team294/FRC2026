// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.EndType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ClimberConstants.ClimberLimit;
import frc.robot.Constants.ClimberConstants.ClimberSide;
import frc.robot.commands.ClimberSetPosition;
import frc.robot.commands.HoodSetAngle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.LEDEventUtil;

public class LowerHoodAndClimber extends SequentialCommandGroup {

  /**
   * Lowers hood and climber to their minimums, flashing LEDS red while lowering and
   * displaying solid green when it is safe to go under the trench (both are down)
   * @param shooter Shooter subsystem
   * @param climber Climber subsystem
  */
  public LowerHoodAndClimber(Shooter shooter, Climber climber) {
    addCommands(
      parallel(
        new ClimberSetPosition(ClimberConstants.ClimberPosition.LOWER_LIMIT, climber),
        new HoodSetAngle(ShooterConstants.hoodMinimumAngle, EndType.waitForSetPoint, shooter)
      ),
      new WaitUntilCommand(() -> climber.isSideLimitReached(ClimberSide.LEFT, ClimberLimit.LOWER) || climber.isSideLimitReached(ClimberSide.RIGHT, ClimberLimit.LOWER)),
      new InstantCommand(() -> LEDEventUtil.sendEvent(LEDEventUtil.LEDStripEvents.TRENCH_SAFE))
    );
  }
}
