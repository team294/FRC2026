// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

import java.util.function.Supplier;

import frc.robot.Constants.*;
import frc.robot.Constants.FeederConstants.SpindexerConstants;
import frc.robot.Constants.FeederConstants.UptakeConstants;
import frc.robot.commands.*;
import frc.robot.commands.sequences.*;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final Timer teleopTimer = new Timer();
  private final AllianceSelection allianceSelection = new AllianceSelection();
  private final ShiftHelper shiftHelper = new ShiftHelper(allianceSelection.getAlliance(), teleopTimer);
  private final Field field = new Field(allianceSelection);

  // Define robot subsystems
  private final DriveTrain driveTrain = new DriveTrain(allianceSelection, field);
  private final Intake intake = new Intake("Intake");
  // private final LED led = new LED(Ports.CANdle, Ports.CANbusMain, "LED", teleopTimer);
  private final Feeder feeder = new Feeder("Feeder");
  private final Shooter shooter = new Shooter("Shooter");
  private final Turret turret = new Turret("Turret");
  private final Climber climber = new Climber("Climber");

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
  // TODO (LED) uncomment when LEDs are ready on the robot
  // private final RepeatCommand repeatedLEDAnimateRobotDisabled = (new LEDAnimateRobotDisabled(led)).repeatedly();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DataLogUtil.writeMessageEcho("RobotContainer: Constructor, Version =", Constants.bcrRobotCodeVersion);
    SignalLogger.enableAutoLogging(false);

    // Start LEDEventUtil
    // TODO (LED) uncomment when LEDs are ready on the robot
    // LEDEventUtil.initialize(led, repeatedLEDAnimateRobotDisabled);

    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences");
    }

    // Set the Choreo trajectory flipper to the 2026 field orientation.
    // When the alliance is Red, Choreo will automatically flip Blue trajectories to Red.
    // All trajectories should be defined based on the Blue alliance perspective.
    ChoreoAllianceFlipUtil.setYear(2026);

    configureXboxButtons();     // configure Xbox controller
    configureJoystickButtons(); // configure joysticks
    configureCopanel();         // configure copanel
    configureTriggers();
    configureElastic();

    driveTrain.setDefaultCommand(new DriveWithJoysticksAdvanced(leftJoystick, rightJoystick, allianceSelection, driveTrain));
    turret.setDefaultCommand(new TurretAutoAim(turret, driveTrain, allianceSelection));
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
    SmartDashboard.putData("Drive Calibration 0.1", new DriveCalibration(0.0, 0.1, 5.0, 100000, driveTrain));
    SmartDashboard.putData("Drive Calibration 0.25", new DriveCalibration(0.0, 0.25, 5.0, 100000, driveTrain));
    SmartDashboard.putData("Drive Calibration 0.4", new DriveCalibration(0.0, 0.4, 5.0, 100000, driveTrain));
    SmartDashboard.putData("Drive Calibration 0.55", new DriveCalibration(0.0, 0.55, 5.0, 100000, driveTrain));
    SmartDashboard.putData("Drive Calibration 0.7", new DriveCalibration(0.0, 0.7, 5.0, 100000, driveTrain));
    SmartDashboard.putData("Drive Calibration 0.85", new DriveCalibration(0.0, 0.85, 5.0, 100000, driveTrain));
    SmartDashboard.putData("Drive Calibration 1.0", new DriveCalibration(0.0, 1.0, 5.0, 100000, driveTrain));
    SmartDashboard.putData("Drive Turn Calibration", new DriveTurnCalibration(0.2, 5.0, 0.2 / 5.0, driveTrain));
    SmartDashboard.putData("Drive Percent Speed", new DrivePercentSpeed(driveTrain));
    SmartDashboard.putData("Drive Straight", new DriveStraight(false, false, true, driveTrain));

    // Vision
    SmartDashboard.putData("Vision Enable Odometry Updates", new VisionOdometryStateSet(true, driveTrain));
    SmartDashboard.putData("Vision Disable Odometry Updates", new VisionOdometryStateSet(false, driveTrain));

    // Stop
    SmartDashboard.putData(INTAKE_ROLLER_CMD_STOP.value, new IntakeRollerStop(intake));
    SmartDashboard.putData(INTAKE_DEPLOY_CMD_STOP.value, new IntakeDeployStop(intake));
    SmartDashboard.putData(INTAKE_CMD_STOP.value, new IntakeStop(intake));
    SmartDashboard.putData(SPINDEXER_CMD_STOP.value, new SpindexerStop(feeder));
    SmartDashboard.putData(UPTAKE_CMD_STOP.value, new UptakeStop(feeder));
    SmartDashboard.putData(FEEDER_CMD_STOP.value, new FeederStop(feeder));
    SmartDashboard.putData(SHOOTER_CMD_STOP.value, new ShooterStop(shooter));
    SmartDashboard.putData(TURRET_CMD_STOP.value, new TurretStop(turret));
    SmartDashboard.putData(STOP_ALL_MOTORS.value, new StopAllMotors(driveTrain, intake, feeder, shooter, climber));

    // Set Percent
    SmartDashboard.putData(INTAKE_ROLLER_CMD_SET_PERCENT.value, new IntakeRollerSetPercent(false, intake));
    SmartDashboard.putData(INTAKE_DEPLOY_CMD_SET_PERCENT.value, new IntakeDeploySetPercent(false, intake));
    SmartDashboard.putData(SPINDEXER_CMD_SET_PERCENT.value, new SpindexerSetPercent(false, feeder));
    SmartDashboard.putData(UPTAKE_CMD_SET_PERCENT.value, new UptakeSetPercent(false, feeder));
    SmartDashboard.putData(SHOOTER_CMD_SET_PERCENT.value, new ShooterSetPercent(shooter));
    SmartDashboard.putData(TURRET_CMD_SET_PERCENT.value, new TurretSetPercent(turret));

    // Set Velocity
    SmartDashboard.putData(SPINDEXER_CMD_SET_VELOCITY.value, new SpindexerSetVelocity(feeder));
    SmartDashboard.putData(UPTAKE_CMD_SET_VELOCITY.value, new UptakeSetVelocity(feeder));
    SmartDashboard.putData(SHOOTER_CMD_SET_VELOCITY.value, new ShooterSetVelocity(VelocityType.runForever, shooter));

    // Set Angle
    SmartDashboard.putData(INTAKE_DEPLOY_CMD_SET_ANGLE.value, new IntakeDeploySetAngle(intake));
    SmartDashboard.putData(TURRET_CMD_SET_ANGLE.value, new TurretSetAngle(EndType.runForever, turret));

    // Set Position

    // Score Commands

    SmartDashboard.putData(SHOOTER_CMD_SCORE_PREP.value + " Manual Input", new ShooterScorePrep(EndType.runForever, shooter, driveTrain, field));
    SmartDashboard.putData(SHOOTER_CMD_SCORE_PREP.value + " Self-Calculate", new ShooterScorePrep(false, EndType.runForever, shooter, driveTrain, field));

    // Calibration Ramp
    
    // Calibrate Manual
    SmartDashboard.putData(INTAKE_DEPLOY_CMD_CALIBRATE_MANUAL.value, new IntakeDeployCalibrateManual(intake));
    SmartDashboard.putData(TURRET_CMD_RECALIBRATE_MANUAL.value, new TurretManualCalibration(turret, true));
    SmartDashboard.putData(TURRET_CMD_RECALIBRATE_AUTOMATIC.value, new TurretManualCalibration(turret, false));

    // Sequences
    SmartDashboard.putData(INTAKING_CMD_SEQUENCE.value, new IntakeSequence(intake));
    SmartDashboard.putData(FEED_SHOOTER_CMD_SEQUENCE.value, new FeederIntakeFeedShooterSequence(feeder, intake, shooter, turret));
    SmartDashboard.putData(PREP_SHOOTER_CMD_SEQUENCE.value, new ShooterTurretScorePrepSequence(false, EndType.runForever, turret, shooter, driveTrain, field));

    // Climber
    SmartDashboard.putData(CLIMBER_CMD_STOP.value, new ClimberStop(climber));
    SmartDashboard.putData(CLIMBER_CMD_SET_PERCENT.value, new ClimberSetPercent(climber));
    SmartDashboard.putData(CLIMBER_CMD_STOP_LEFT.value, new ClimberSingleStop(ClimberConstants.SIDE_LEFT, climber));
    SmartDashboard.putData(CLIMBER_CMD_STOP_RIGHT.value, new ClimberSingleStop(ClimberConstants.SIDE_RIGHT, climber));
    SmartDashboard.putData(CLIMBER_CMD_SET_PERCENT_LEFT.value, new ClimberSingleSetPercent(ClimberConstants.SIDE_LEFT, climber));
    SmartDashboard.putData(CLIMBER_CMD_SET_PERCENT_RIGHT.value, new ClimberSingleSetPercent(ClimberConstants.SIDE_RIGHT, climber));
    SmartDashboard.putData("Climber L Down", new ClimberSingleSetPercent(-0.1, ClimberConstants.SIDE_LEFT, climber));

    SmartDashboard.putData(HOOD_CMD_SET_VOLTAGE_BOTH.value, new HoodSetVoltage(shooter, 0));
    SmartDashboard.putData(HOOD_CMD_SET_VOLTAGE_LEFT.value, new HoodSetVoltage(shooter, 1));
    SmartDashboard.putData(HOOD_CMD_SET_VOLTAGE_RIGHT.value, new HoodSetVoltage(shooter, 2));
    SmartDashboard.putData(HOOD_CMD_SET_PERCENT_LEFT.value, new HoodSetPercent(true, shooter, 1));
    SmartDashboard.putData(HOOD_CMD_SET_PERCENT_RIGHT.value, new HoodSetPercent(true, shooter, 2));
    SmartDashboard.putData(HOOD_CMD_SET_ANGLE_FOREVER.value, new HoodSetAngle(EndType.runForever, shooter));
    SmartDashboard.putData(HOOD_CMD_SET_ANGLE_WAIT.value, new HoodSetAngle(EndType.waitForSetPoint, shooter));
    SmartDashboard.putData(HOOD_CMD_STOP_BOTH.value, new HoodSetVoltage(shooter, 0, 0));
    SmartDashboard.putData(HOOD_CMD_STOP_LEFT.value, new HoodSetVoltage(shooter, 0, 1));
    SmartDashboard.putData(HOOD_CMD_STOP_RIGHT.value, new HoodSetVoltage(shooter, 0, 2));

    // Autos
    SmartDashboard.putData("Autonomous Run Auto Now", autoSelection.scheduleAutoCommand());
  }
 
  /**
   * Define Xbox buttons and controls.
   */
  private void configureXboxButtons() {
    // Triggers for all Xbox buttons
    Trigger xbLT = xboxController.leftTrigger();
    Trigger xbRT = xboxController.rightTrigger();
    // Trigger xbA = xboxController.a();
    // Trigger xbB = xboxController.b();
    // Trigger xbY = xboxController.y();
    // Trigger xbX = xboxController.x();
    Trigger xbLB = xboxController.leftBumper();
    Trigger xbRB = xboxController.rightBumper();
    Trigger xbBack = xboxController.back();
    Trigger xbStart = xboxController.start();
    Trigger xbPOVUp = xboxController.povUp();
    Trigger xbPOVRight = xboxController.povRight();
    // Trigger xbPOVLeft = xboxController.povLeft();
    Trigger xbPOVDown = xboxController.povDown();
    Trigger xbLJoystickTrigger = xboxController.leftStick();
    // Trigger xbRJoystickTrigger = xboxController.rightStick();

    // ex: xbA.onTrue(new command(param1, param2, etc));

    xbLT.onTrue(new IntakeSequence(intake));
    xbLB.onTrue(new IntakeRollerStop(intake));

    // Agitate while shooting
    xbRT.whileTrue(parallel(
      new ShooterTurretScorePrepSequence(false, EndType.runForever, turret, shooter, driveTrain, field),
      new InstantCommand(() -> intake.setAgitateDuringFeed(true))
    ));
    xbRT.onFalse(parallel(
      new FeederStop(feeder),
      new IntakeDeploySetAngle(IntakeConstants.IntakeDeployAngle.INTAKE, intake),
      new InstantCommand(() -> intake.setAgitateDuringFeed(false))
    ));

    // Intake while shooting
    xbRB.whileTrue(parallel(
      new ShooterTurretScorePrepSequence(false, EndType.runForever, turret, shooter, driveTrain, field),
      new InstantCommand(() -> intake.setAgitateDuringFeed(true))
    ));
    xbRT.onFalse(parallel(
      new FeederStop(feeder),
      new IntakeSequence(intake),
      new InstantCommand(() -> intake.setAgitateDuringFeed(false))
    ));

    xbLJoystickTrigger.onTrue(new IntakeDeployManualControl(xboxController, intake));

    xbBack.onTrue(new StopAllMotors(driveTrain, intake, feeder, shooter, climber));
    xbStart.onTrue(new TurretToggleAutoAim(turret));

    xbPOVUp.whileTrue(new ClimberSetPercent(ClimberConstants.maxManualPercentOutput, climber));
    xbPOVDown.whileTrue(new ClimberSetPercent(-ClimberConstants.maxManualPercentOutput, climber));
    xbPOVRight.onTrue(new HoodSetAngle(ShooterConstants.hoodMinimumAngle, EndType.immediatelyEnd, shooter));
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

    left[1].onTrue(new IntakeSequence(intake));

    // left[2].whileTrue(new FeedShooter(intake, feeder));
    left[2].whileTrue(new FeederIntakeFeedShooterSequence(feeder, intake, shooter, turret));

    // right[1].whileTrue(new ShooterSetVelocity(ShooterConstants.shootRPM, VelocityType.runForever, shooter));
    right[1].whileTrue(new DriveAlongNearestWall(driveTrain));

    right[2].onTrue(new DriveResetDriverAngle(driveTrain));
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

    // Send intake deploy in/out manually while held, stop when released
    coP[1].whileTrue(new IntakeDeploySetPercent(IntakeConstants.deployMaxManualPercentOutput, false, intake));
    coP[2].whileTrue(new IntakeDeploySetPercent(-IntakeConstants.deployMaxManualPercentOutput, false, intake));

    // Spin rollers inwards/outwards while held, stop when released
    coP[3].whileTrue(new IntakeRollerSetPercent(IntakeConstants.rollerIntakePercent, false, intake));
    coP[4].whileTrue(new IntakeRollerSetPercent(-IntakeConstants.rollerIntakePercent, false, intake));

    // Spin uptake then spindexer to feed shooter while held, stop when released
    coP[5].onTrue(
      sequence(
        new UptakeSetVelocity(UptakeConstants.uptakeShootRPM, feeder),
        new SpindexerSetVelocity(SpindexerConstants.spindexerShootRPM, feeder)
      )
    );
    coP[5].onFalse(new FeederStop(feeder));

    // Spin uptake then spindexer away from shooter while held, stop when released
    coP[6].onTrue(
      sequence(
        new UptakeSetVelocity(-UptakeConstants.uptakeShootRPM, feeder),
        new SpindexerSetVelocity(-SpindexerConstants.spindexerShootRPM, feeder)
      )
    );
    coP[6].onFalse(new FeederStop(feeder));

    // Reset Pose
    coP[7].onTrue(either(
      new DriveResetPose(180, false, driveTrain), 
      new DriveResetPose(0, false, driveTrain), 
      () -> allianceSelection.getAlliance() == Alliance.Red));

    // // Manually spin turret CW/CCW while held, stopping when released
    coP[9].whileTrue(new TurretSetPercent(turret, -TurretConstants.maxManualPercentOutput));
    coP[10].whileTrue(new TurretSetPercent(turret, TurretConstants.maxManualPercentOutput));

    // Manually move the hood up/down while held, stopping when released
    coP[11].whileTrue(new HoodSetPercent(false, ShooterConstants.hoodMaximumPercentOutput, shooter, 0));
    coP[12].whileTrue(new HoodSetPercent(false, -ShooterConstants.hoodMaximumPercentOutput, shooter, 0));

    // Manually spin shooter out/in while held, stopping when released
    coP[13].whileTrue(new ShooterSetVelocity(ShooterConstants.shootRPM, VelocityType.runForever, shooter));
    coP[14].whileTrue(new ShooterSetVelocity(ShooterConstants.reverseShootRPM, VelocityType.runForever, shooter));

    // Manually move climber up/down while held, stopping when released
    coP[15].whileTrue(new ClimberSetPercent(ClimberConstants.maxManualPercentOutput, climber));
    coP[16].whileTrue(new ClimberSetPercent(-ClimberConstants.maxManualPercentOutput, climber));

    // Manually calibrate climber, assuming it's at the lower limit
    // coP[18].onTrue(new ClimberManualCalibration(climber));

    // // Manually recalibrate Turret, assuming it's facing straight out of back of robot
    coP[19].onTrue(new TurretManualCalibration(turret, true));

    // Manually recalibrate intake deploy, assuming it's at intaking position
    coP[20].onTrue(new IntakeDeployCalibrateManual(IntakeConstants.IntakeDeployAngle.INTAKE.value, intake));
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
    shiftHelper.periodic();
    autoSelection.periodic();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    // Do not log the word "Init" here, as it it affects the Excel macro
    DataLogUtil.writeMessageEcho("Disabled: Robot disabled");

    driveTrain.stopMotors();             // SAFETY: Turn off any closed loop control that may be running, so the robot does not move when re-enabled
    feeder.stopUptake();
    feeder.stopSpindexer();
    intake.stopIntakeDeploy();
    intake.stopIntakeRoller();
    driveTrain.enableFastLogging(false); // Turn off fast logging, in case it was left on from auto mode
    driveTrain.setVisionForOdometryState(true);

    LEDEventUtil.sendEvent(LEDStripEvents.ROBOT_DISABLED);
    teleopTimer.stop();

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
    // TODO (LED) uncomment when LEDs are ready on the robot
    // CommandScheduler.getInstance().cancel(repeatedLEDAnimateRobotDisabled);
    // LEDEventUtil.sendEvent(LEDStripEvents.NEUTRAL);

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
    // TODO (LED) uncomment when LEDs are ready on the robot
    // CommandScheduler.getInstance().cancel(repeatedLEDAnimateRobotDisabled);
    // LEDEventUtil.sendEvent(LEDStripEvents.NEUTRAL);

    teleopTimer.restart();
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {
  }
}
