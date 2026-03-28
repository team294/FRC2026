// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EndType;
import frc.robot.Constants.IntakeConstants.IntakeDeployAngle;
import frc.robot.commands.IntakeDeploySetAngle;
import frc.robot.commands.autos.components.AutonScore;
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
public class ShootPreload extends SequentialCommandGroup {
  /** Starts with front bumpers on the start line, facing away from drivers. Shoots preload then does nothing else */
  public ShootPreload(EndType endType, Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber, DriveTrain driveTrain, Field field) {
    addCommands(
      either(
        sequence(
          new IntakeDeploySetAngle(IntakeDeployAngle.INTAKE, intake),
          new AutonScore(endType, intake, feeder, turret, shooter, climber, driveTrain, field).withTimeout(8)
        ),
        none(),
        () -> intake.isIntakeDeployCalibrated()
      )
    );
  }
}