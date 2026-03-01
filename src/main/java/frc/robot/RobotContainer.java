// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.LEDEventUtil.LEDStripEvents;

import com.ctre.phoenix6.SignalLogger;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Define key utilities here:
  private final AllianceSelection allianceSelection = new AllianceSelection();
  private final Field field = new Field(allianceSelection);
  private final Timer teleopTimer = new Timer();

  // Define robot subsystems
  private final DriveTrain driveTrain = new DriveTrain(allianceSelection, field);
  // private final Intake intake = new Intake("Intake");
  private final LED led = new LED(Ports.CANdle, Ports.CANbusMain, "LED", teleopTimer);
  // private final Feeder feeder = new Feeder("Feeder");
  // private final Shooter shooter = new Shooter("Shooter");

  // Define controllers
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  private final Joystick coPanel = new Joystick(OIConstants.usbCoPanel);
  private final CommandXboxController xboxController = new CommandXboxController(OIConstants.usbXboxController);

  // Define other utilities
  private final TrajectoryCache trajectoryCache = new TrajectoryCache();
  private final AutoSelection autoSelection = new AutoSelection(rightJoystick, trajectoryCache, allianceSelection, field, driveTrain);
  
  // this command repeats until interrupted/cancelled (we cancel it in teleopInit() and autonomousInit()).
  // the command is created here to be passed into LEDEventUtil's initialize method
  private final RepeatCommand repeatedLEDAnimateRobotDisabled = (new LEDAnimateRobotDisabled(led)).repeatedly();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DataLogUtil.writeMessageEcho("RobotContainer: Constructor, Version =", Constants.bcrRobotCodeVersion);
    SignalLogger.enableAutoLogging(false);

    // Start LEDEventUtil
    LEDEventUtil.initialize(led, repeatedLEDAnimateRobotDisabled);

    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences");
    }

    // Set the Choreo trajectory flipper to the 2026 field orientation.
    // When the alliance is Red, Choreo will automatically flip Blue trajectories to Red.
    // All trajectories should be defined based on the Blue alliance perspective.
    ChoreoAllianceFlipUtil.setYear(2026);

    configureButtonBindings();
    configureElastic();

    driveTrain.setDefaultCommand(new DriveWithJoysticksAdvanced(leftJoystick, rightJoystick, allianceSelection, driveTrain));
    // driveTrain.setDefaultCommand(new DriveWithJoysticks(leftJoystick, rightJoystick, allianceSelection, driveTrain));
    // driveTrain.setDefaultCommand(new DriveWithController(driveTrain, xboxController, allianceSelection));

    // Set initial robot position on field. This takes place a while after the drivetrain is created, so after any CANbus delays.
    // Set initial location to (0,0) and facing away from driver (regardless of alliance color).
    driveTrain.resetPose(new Pose2d(0.0, 0.0, 
      allianceSelection.getAlliance() == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero));
  }

  private void configureElastic() {
    // Sticky Faults
    RobotPreferences.showStickyFaultsOnElastic();
    SmartDashboard.putData("Clear Sticky Faults", new StickyFaultsClear());

    // DriveTrain
    SmartDashboard.putData("Drive FOC On", new DriveSetFOC(true, driveTrain));
    SmartDashboard.putData("Drive FOC Off", new DriveSetFOC(false, driveTrain));
    SmartDashboard.putData("Drive Toggle Coast", new DriveToggleCoastMode(driveTrain));
    SmartDashboard.putData("Drive Reset Pose", new DriveResetPose(driveTrain));
    SmartDashboard.putData("Drive Reset Driver Angle", new DriveResetDriverAngle(driveTrain));
    SmartDashboard.putData("Drive To Pose Absolute", new DriveToPose(CoordType.kAbsolute, driveTrain));
    SmartDashboard.putData("Drive To Pose Relative", new DriveToPose(CoordType.kRelative, driveTrain));
    SmartDashboard.putData("Drive 6m Fwd", new DriveToPose(CoordType.kRelative, () -> new Pose2d(6.0, 0.0, new Rotation2d(0.0)),
      SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, 
      TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
      true, true, driveTrain));
    SmartDashboard.putData("Drive Calibration", new DriveCalibration(0.0, 0.5, 5.0, 0.1, driveTrain));
    SmartDashboard.putData("Drive Turn Calibration", new DriveTurnCalibration(0.2, 5.0, 0.2 / 5.0, driveTrain));
    SmartDashboard.putData("Drive Percent Speed", new DrivePercentSpeed(driveTrain));
    SmartDashboard.putData("Drive Straight", new DriveStraight(false, false, false, driveTrain));

    // Vision
    SmartDashboard.putData("Vision Enable Odometry Updates", new VisionOdometryStateSet(true, driveTrain));
    SmartDashboard.putData("Vision Disable Odometry Updates", new VisionOdometryStateSet(false, driveTrain));

    // Intake
    // SmartDashboard.putData("Intake Set Percent", new IntakeSetPercent(intake));
    // SmartDashboard.putData("Intake Stop", new IntakeStop(intake));

    // Feeder
    // SmartDashboard.putData("Feeder Set Percent", new FeederSetPercent(feeder));
    // SmartDashboard.putData("Feeder Stop", new FeederStop(feeder));
    // SmartDashboard.putData("Feeder Set Velocity", new FeederSetVelocity(FeederSetVelocity.VelocityType.runForever, feeder));
    // SmartDashboard.putData("Feeder Set Velocity 500 RPM", new FeederSetVelocity(500.0, FeederSetVelocity.VelocityType.runForever, feeder));
    // SmartDashboard.putData("Feeder set PIDSVA", new FeederSetPIDSVA(feeder));
    
    // Shooter
    // SmartDashboard.putData("Shooter Set Percent", new ShooterSetPercent(shooter));
    // SmartDashboard.putData("Shooter Set Velocity 500 rpm", new ShooterSetVelocity(500, VelocityType.runForever, shooter));
    // SmartDashboard.putData("Shooter Set Velocity", new ShooterSetVelocity(VelocityType.runForever, shooter));
    // SmartDashboard.putData("Shooter Stop", new ShooterStop(shooter));
    // SmartDashboard.putData("Shooter set PIDSVA", new ShooterSetPIDSVA(shooter));

    // Autos
    SmartDashboard.putData("Autonomous Run Auto Now", autoSelection.scheduleAutoCommand());
  }
 
   private void configureButtonBindings() {
    configureXboxButtons();     // configure Xbox controller
    configureJoystickButtons(); // configure joysticks
    configureCopanel();         // configure copanel
    configureTriggers();
  }

  /**
   * Define Xbox buttons and controls.
   */
  private void configureXboxButtons() {
    // Triggers for all Xbox buttons
    // Trigger xbLT = xboxController.leftTrigger();
    // Trigger xbRT = xboxController.rightTrigger();
    // Trigger xbA = xboxController.a();
    // Trigger xbB = xboxController.b();
    // Trigger xbY = xboxController.y();
    // Trigger xbX = xboxController.x();
    // Trigger xbLB = xboxController.leftBumper();
    // Trigger xbRB = xboxController.rightBumper();
    // Trigger xbBack = xboxController.back();
    // Trigger xbStart = xboxController.start();
    // Trigger xbPOVUp = xboxController.povUp();
    // Trigger xbPOVRight = xboxController.povRight();
    // Trigger xbPOVLeft = xboxController.povLeft();
    // Trigger xbPOVDown = xboxController.povDown();
    // Trigger xbRJoystickTrigger = xboxController.rightStick();
    // Trigger xbLJoystickTrigger = xboxController.leftStick();

    // ex: xbA.onTrue(new command(param1, param2, etc));
  }

  /**
   * Define driver joystick button mappings.
   */
  public void configureJoystickButtons() {
    JoystickButton[] left = new JoystickButton[3];
    JoystickButton[] right = new JoystickButton[3];

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    // ex: left[1].onTrue(new command(param1, param2, etc));
    // left[1].onTrue(new IntakeStop(intake));
    left[2].onTrue(new DriveResetPose(driveTrain));

    // set intake to 100% (in) if not set to 
    // 100% and stop intake if it is set to 100%
    // right[1].onTrue(either(
    //   new IntakeSetPercent(IntakeConstants.intakePercent, intake),
    //   new IntakeStop(intake),
    //   () -> (intake.getTargetPercentOutput() != 1)
    // ));

    // set intake to -100% (out) if not set to
    // -100% and stop intake if it is set to -100%
    // right[2].onTrue(either(
    //   new IntakeSetPercent(IntakeConstants.reverseIntakePercent, intake),
    //   new IntakeStop(intake),
    //   () -> (intake.getTargetPercentOutput() != -1)
    // ));
  }

  /**
   * Define Copanel button mappings.
   *  
   *  1  3  5  8
   *  2  4  6  8
   *      
   *  9  11 13 7
   *  10 12 14 7
   * 
   *  15 17 19
   *  16 18 20
   */
  public void configureCopanel() {
    JoystickButton[] coP = new JoystickButton[21];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // ex: coP[1].onTrue(new command(param1, param2, etc));
    
    // Reset Pose
    coP[7].onTrue(either(
      new DriveResetPose(180, false, driveTrain), 
      new DriveResetPose(0, false, driveTrain), 
      () -> allianceSelection.getAlliance() == Alliance.Red));
  }

  private void configureTriggers() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return none();
    return autoSelection.getAutoCommand();
  }

  
  /**
   * Method called once every scheduler cycle (20 ms).
   */
  public void robotPeriodic(){
    DataLogUtil.advanceLogRotation();
    allianceSelection.periodic();
    autoSelection.periodic();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    // Do not log the word "Init" here, as it it affects the Excel macro
    DataLogUtil.writeMessageEcho("Disabled: Robot disabled");

    driveTrain.stopMotors();             // SAFETY: Turn off any closed loop control that may be running, so the robot does not move when re-enabled
    // intake.stopIntakeMotor();
    // feeder.stopFeederMotor();
    // shooter.stopShooterMotor();
    driveTrain.enableFastLogging(false); // Turn off fast logging, in case it was left on from auto mode
    driveTrain.setVisionForOdometryState(true);

    LEDEventUtil.sendEvent(LEDStripEvents.ROBOT_DISABLED);
    teleopTimer.stop();
    teleopTimer.reset();

    SignalLogger.stop();
  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
    // Check for CAN bus error (to prevent the issue that caused us to be eliminated in 2020!)
    if (driveTrain.canBusError()) {
      RobotPreferences.recordStickyFaults("CAN Bus");
    }
  }
  
  /**
   * Method called when auto mode is initialized/enabled.
   */
  public void autonomousInit() {
    DataLogUtil.writeMessageEcho("Auto: Mode Init");

    driveTrain.setDriveModeCoast(false);
    driveTrain.setVisionForOdometryState(true);

    // we have to cancel this command to clear the disabled
    // animations off the LEDs because they're repeated forever
    CommandScheduler.getInstance().cancel(repeatedLEDAnimateRobotDisabled);
    LEDEventUtil.sendEvent(LEDStripEvents.NEUTRAL);

    // NOTE: Do not reset the gyro or encoder here!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.
  }

  /**
   * Method called once every scheduler cycle when auto mode is initialized/enabled.
   */
  public void autonomousPeriodic() {
    
  }

  /**
   * Method called when teleop mode is initialized/enabled.
   */
  public void teleopInit() {
    DataLogUtil.writeMessageEcho("Teleop: Mode Init");

    driveTrain.setDriveModeCoast(false); // Set drive mode to brake mode
    driveTrain.enableFastLogging(false); // Turn off fast logging, in case it was left on from auto mode
    driveTrain.setVisionForOdometryState(true);

    // we have to cancel this command to clear the disabled
    // animations off the LEDs because they're repeated forever
    CommandScheduler.getInstance().cancel(repeatedLEDAnimateRobotDisabled);
    LEDEventUtil.sendEvent(LEDStripEvents.NEUTRAL);

    teleopTimer.start();
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {
  }
}
