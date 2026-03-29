// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.EndType;
import frc.robot.commands.ClimberSetPercent;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.sequences.FeederIntakeFeedShooterSequence;
import frc.robot.commands.sequences.ShooterTurretScorePrepSequence;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.Field;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonScore extends SequentialCommandGroup {
  /** Creates a new AutonScore. */
  public AutonScore(EndType endType, Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber, DriveTrain driveTrain, Field field) {
    addCommands(
      new DataLogMessage(false, "AutonScore: start"),
      parallel(
        new ShooterTurretScorePrepSequence(false, endType, turret, shooter, driveTrain, field),
        new FeederIntakeFeedShooterSequence(feeder, intake, shooter, turret),
        new ClimberSetPercent(ClimberConstants.maxManualPercentOutput, climber)
        // TODO end condition for not seeing balls for extendend period of time? then put hood down
      )
    );
  }
}
