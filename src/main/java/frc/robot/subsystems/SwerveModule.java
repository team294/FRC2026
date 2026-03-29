// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utilities.*;

import static frc.robot.utilities.StringUtil.*;

public class SwerveModule {
  private final String swName;  // Name for this swerve module
  
  private final double turningOffsetDegrees;

  // Drive motor objects
  private final TalonFX driveMotor;
	private final TalonFXConfigurator driveMotorConfigurator;
	private TalonFXConfiguration driveMotorConfig;
	private VoltageOut driveVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private VelocityVoltage driveVelocityControl = new VelocityVoltage(0.0).withEnableFOC(false);

	// Drive motor signals and sensors
	private final StatusSignal<Voltage> driveMotorSupplyVoltage;			// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Temperature> driveMotorTemp;				    // Motor temperature, in degrees Celsius
	private final StatusSignal<Double> driveDutyCycle;				        // Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Current> driveStatorCurrent;		        // Motor stator current, in amps (positive = forward, negative = reverse)
	private final StatusSignal<Angle> driveEncoderPostion;            // Encoder position, in pinion rotations
	private final StatusSignal<AngularVelocity> driveEncoderVelocity; // Encoder velocity, in pinion rotations/second

  // Turning motor objects
  private final TalonFX turningMotor;
	private final TalonFXConfigurator turningMotorConfigurator;
	private TalonFXConfiguration turningMotorConfig;
	private VoltageOut turningVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private PositionVoltage turningPositionControl = new PositionVoltage(0.0).withEnableFOC(false);

	// Turning motor signals and sensors
	private final StatusSignal<Temperature> turningMotorTemp;				    // Motor temperature, in degrees Celsius
	private final StatusSignal<Double> turningDutyCycle;				        // Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Current> turningStatorCurrent;		        // Motor stator current, in amps (positive = forward, negative = reverse)
	private final StatusSignal<Angle> turningEncoderPosition;           // Encoder position, in pinion rotations
	private final StatusSignal<AngularVelocity> turningEncoderVelocity; // Encoder velocity, in pinion rotations/second

  // Variables to track motor information
  private boolean isInCoastMode;  // Current NeutralMode setting for the drive and turning motors, with true = coast mode, false = brake mode

  // CANcoder objects
  private final CANcoder turningCanCoder;
  private final CANcoderConfigurator turningCanCoderConfigurator;
  private CANcoderConfiguration turningCanCoderConfig;
	
  // CANcoder signals and sensors
	private final StatusSignal<Angle> turningCanCoderPosition;            // CANcoder position, in CANcoder rotations
	private final StatusSignal<AngularVelocity> turningCanCoderVelocity;  // CANcoder velocity, in CANcoder rotations/second

  // Variables for encoder zeroing
  private double driveEncoderZero = 0;    // Reference raw encoder reading for drive motor encoder. Calibration sets this to zero.
  private double cancoderZero = 0;        // Reference raw encoder reading for CanCoder. Calibration sets this to the absolute position from RobotPreferences.
  private double turningEncoderZero = 0;  // Reference raw encoder reading for turning motor encoder. Calibration sets this to match the CanCoder.

  // Controller for drive motor speed
  private final SimpleMotorFeedforward driveFeedforward;
  private double priorDriveVelocity = 0;  // Last requested drive speed (m/sec)
  private boolean inVelocityMode = true;  // true = last drive request was velocity control (or stopped), false = last drive request was direct non-zero voltage control

  // Data logging variables
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogCCAngle;
  private final DoubleLogEntry dLogCCTurnRate;
  private final DoubleLogEntry dLogFXAngle;
  private final DoubleLogEntry dLogFXTurnRate;
  private final DoubleLogEntry dLogTurnTemp;
  private final DoubleLogEntry dLogTurnOutput;
  private final DoubleLogEntry dLogDriveTemp;
  private final DoubleLogEntry dLogDriveOutput;
  private final DoubleLogEntry dLogDriveDist;
  private final DoubleLogEntry dLogDriveSpeed;
  private final DoubleLogEntry dLogDriveCurrent;


  /**
   * Constructs a SwerveModule.
   * @param swName The name of this swerve module, for use in Elastic and logging
   * @param driveMotorAddress The CANbus address of the drive motor
   * @param turningMotorAddress The CANbus address of the turning motor
   * @param cancoderAddress The CANbus address of the turning encoder
   * @param driveMotorInverted true = invert drive motor direction, false = do not invert
   * @param turningMotorInverted true = invert turning motor direction, false = do not invert
   * @param cancoderReversed true = the CANcoder is reversed, false = not reversed
   * @param turningOffsetDegrees Offset degrees in the turning motor to point to the front of the robot. Value is the desired encoder zero point, in absolute magnet position reading.
   * @param kVm Drive motor kV multiplier to account for small differences between the 4 swerve modules on the robot. The drive motor kV = kVDriveAvg * kVm.
   */
  public SwerveModule(String swName, int driveMotorAddress, int turningMotorAddress, int cancoderAddress,
      boolean driveMotorInverted, boolean turningMotorInverted, boolean cancoderReversed, double turningOffsetDegrees,
      double kVm) {
    this.swName = swName;
    
    this.turningOffsetDegrees = turningOffsetDegrees;

    // Create feed forward model for drive motor
    driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.kSDrive, SwerveConstants.kVDriveAvg*kVm, SwerveConstants.kADrive);
    
    // Create motor, encoder, signal, and sensor objects for drive motor
    driveMotor = new TalonFX(driveMotorAddress, Ports.CANbusCANivore);
    driveMotorConfigurator = driveMotor.getConfigurator();
    driveMotorSupplyVoltage = driveMotor.getSupplyVoltage();
	  driveMotorTemp = driveMotor.getDeviceTemp();
	  driveDutyCycle = driveMotor.getDutyCycle();
	  driveStatorCurrent = driveMotor.getStatorCurrent();
	  driveEncoderPostion = driveMotor.getPosition();
	  driveEncoderVelocity = driveMotor.getVelocity();

    // Create motor, encoder, signal, and sensor objects for turning motor
    turningMotor = new TalonFX(turningMotorAddress, Ports.CANbusCANivore);
    turningMotorConfigurator = turningMotor.getConfigurator();
	  turningMotorTemp = turningMotor.getDeviceTemp();
	  turningDutyCycle = turningMotor.getDutyCycle();
	  turningStatorCurrent = turningMotor.getStatorCurrent();
	  turningEncoderPosition = turningMotor.getPosition();
	  turningEncoderVelocity = turningMotor.getVelocity();

    // Create CANcoder
    turningCanCoder = new CANcoder(cancoderAddress, Ports.CANbusCANivore);
    turningCanCoderConfigurator = turningCanCoder.getConfigurator();
    turningCanCoderPosition = turningCanCoder.getPosition();
    turningCanCoderVelocity = turningCanCoder.getVelocity();

    // **** Setup drive motor configuration

 		// Start with factory default TalonFX configuration
		driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.MotorOutput.Inverted = driveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;  // Boot to brake mode, so robot starts quickly in auto
    driveMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;
		driveMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    // If the current is above the supply current limit for the threshold time, the current is limited to the lower limit.
    // This is configured to prevent the breakers from tripping.
    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;        // Upper limit for the current, in amps
    driveMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;   // Lower limit for the current, in amps
    driveMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 2.0;     // Threshold time, in seconds (was 0.1s, changed to 2.0s)
    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Configure drive encoder
		driveMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Configure PID for VelocityVoltage control
    // NOTE: In Phoenix 6, slots are selected in the ControlRequest (ex. PositionVoltage.Slot)
    driveVelocityControl.Slot = 0;
    driveVelocityControl.OverrideBrakeDurNeutral = true;
    driveMotorConfig.Slot0.kP = SwerveConstants.driveMotorkP;
		driveMotorConfig.Slot0.kI = SwerveConstants.driveMotorkI;
		driveMotorConfig.Slot0.kD = SwerveConstants.driveMotorkD;
		// driveMotorConfig.Slot0.kS = 0.0;
		// driveMotorConfig.Slot0.kV = 0.0;
		// driveMotorConfig.Slot0.kA = 0.0;

    // **** Setup turning motor configuration

    // Start with factory default TalonFX configuration
		turningMotorConfig = new TalonFXConfiguration();
    turningMotorConfig.MotorOutput.Inverted = turningMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		turningMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;  // Boot to brake mode, so robot starts quickly in auto
    turningMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
		turningMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    // If the current is above the supply current limit for the threshold time, the current is limited to the lower limit.
    // This is configured to prevent the breakers from tripping.
    turningMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;        // Upper limit for the current, in amps
    turningMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;   // Lower limit for the current, in amps
    turningMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;     // Threshold time, in seconds
    turningMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Configure turning encoder
		turningMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Configure PID for PositionVoltage control
    // NOTE: In Phoenix 6, slots are selected in the ControlRequest (ex. PositionVoltage.Slot)
    turningPositionControl.Slot = 0;
    turningPositionControl.OverrideBrakeDurNeutral = true;
    turningMotorConfig.Slot0.kP = SwerveConstants.turningMotorkP;
		turningMotorConfig.Slot0.kI = SwerveConstants.turningMotorkI;
		turningMotorConfig.Slot0.kD = SwerveConstants.turningMotorkD;
		// turningMotorConfig.Slot0.kS = 0.0;
		// turningMotorConfig.Slot0.kV = 0.0;
		// turningMotorConfig.Slot0.kA = 0.0;

    // **** Setup CANcoder configuration

 		// Start with factory default CANcoder configuration
    turningCanCoderConfig = new CANcoderConfiguration();
    turningCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;  // 1 makes absolute position unsigned [0, 1); 0.5 makes it signed [-0.5, 0.5), 0 makes it always negative 
    turningCanCoderConfig.MagnetSensor.SensorDirection = cancoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

    // Configure the swerve module motors and encoders
    configSwerveModule();

    // Other configs for drive and turning motors
    setMotorModeCoast(false); // false on boot up, so robot starts quickly in auto. Set to false in autoInit or teleopInit, but true when disabled.

    // Create logfile entries
    dLogCCAngle = new DoubleLogEntry(log, StringUtil.buildString("/DriveTrain/Swerve", swName, "/CCAngleDeg"));
    dLogCCTurnRate = new DoubleLogEntry(log, StringUtil.buildString("/DriveTrain/Swerve", swName, "/CCAngleRateDPS"));
    dLogFXAngle = new DoubleLogEntry(log, StringUtil.buildString("/DriveTrain/Swerve", swName, "/FXSngleDeg"));
    dLogFXTurnRate = new DoubleLogEntry(log, StringUtil.buildString("/DriveTrain/Swerve", swName, "/FXAngleRateDPS"));
    dLogTurnTemp = new DoubleLogEntry(log, StringUtil.buildString("/DriveTrain/Swerve", swName, "/TurnTemp"));
    dLogTurnOutput = new DoubleLogEntry(log, StringUtil.buildString("/DriveTrain/Swerve", swName, "/TurnOutput"));
    dLogDriveTemp = new DoubleLogEntry(log, StringUtil.buildString("/DriveTrain/Swerve", swName, "/DriveTemp"));
    dLogDriveOutput = new DoubleLogEntry(log, StringUtil.buildString("/DriveTrain/Swerve", swName, "/DriveOutput"));
    dLogDriveDist = new DoubleLogEntry(log, StringUtil.buildString("/DriveTrain/Swerve", swName, "/DriveDistMeters"));
    dLogDriveSpeed = new DoubleLogEntry(log, StringUtil.buildString("/DriveTrain/Swerve", swName, "/DriveSpeedMPS"));
    dLogDriveCurrent = new DoubleLogEntry(log, StringUtil.buildString("/DriveTrain/Swerve", swName, "/DriveCurrent"));

    // Prime the DataLog to reduce delay when first enabling the robot
    long timeNow = RobotController.getFPGATime();
    updateSwerveLog(timeNow);
  }

  // ********** Swerve module configuration methods

  /**
   * Configures the motors and encoders, uses default values from the constructor.
   * In general, this should only be called from the constructor when the robot code is starting.
   * However, if the robot browns-out or otherwise partially resets, then this can be used to 
   * force the encoders to have the right calibration and settings, especially the
   * calibration angle for each swerve module.
   * <p><b>NOTE:</b> this procedure includes multiple blocking calls and will delay robot code.
   */
  public void configSwerveModule() {
    // Save setting for NeutralMode
    isInCoastMode = (driveMotorConfig.MotorOutput.NeutralMode == NeutralModeValue.Coast);

 		// Apply configuration to the drive motor.
		// This is a blocking call and will wait up to 50ms-70ms for the config to apply.
		driveMotorConfigurator.apply(driveMotorConfig);

 		// Apply configuration to the turning motor.
		// This is a blocking call and will wait up to 50ms-70ms for the config to apply.
		turningMotorConfigurator.apply(turningMotorConfig);

    // Apply configuration to the CANcoder.
		// This is a blocking call and will wait up to 50ms-70ms for the config to apply.
    turningCanCoderConfigurator.apply(turningCanCoderConfig);

    // NOTE!!! When the Cancoder or TalonFX encoder settings are changed above, then the next call to 
    // getCanCoderDegrees() getTurningEncoderDegrees() may contain an old value, not the value based on 
    // the updated configuration settings above!!!!  The CANBus runs asynchronously from this code, so 
    // sending the updated configuration to the CanCoder/TalonFX and then receiving an updated position 
    // measurement back may take longer than this code.vThe timeouts in the configuration code above 
    // should take care of this, but it does not always wait long enough. So, add a wait time here:
    Wait.waitTime(200);

    // System.out.println(swName + " CanCoder " + getCanCoderDegrees() + " FX " + getTurningEncoderDegrees() + " pre-CAN");
    zeroDriveEncoder();
    // DataLogUtil.writeMessageEcho("SwerveModule ", swName + " pre-CAN", ", Cancoder: ", getCanCoderDegrees(), ", FX: ", getTurningEncoderDegrees());
    calibrateCanCoderDegrees(turningOffsetDegrees);
    // DataLogUtil.writeMessageEcho(true, "SwerveModule ", swName + " post-CAN", ", Cancoder: ", getCanCoderDegrees(), ", FX: ", getTurningEncoderDegrees());
    calibrateTurningEncoderDegrees(getCanCoderDegrees());
    // DataLogUtil.writeMessageEcho(true, "SwerveModule ", swName + " post-FX", ", Cancoder: ", getCanCoderDegrees(), ", FX: ", getTurningEncoderDegrees());
  }

  /**
   * Sets the swerve module to coast or brake mode.
   * <p><b>NOTE:</b> this procedure includes multiple blocking calls and will delay robot code by ~60ms if the mode is changed.
   * However, if setCoast is the same as the current setting, then nothing is sent to the swerve modules and there will not
   * be a delay.
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setMotorModeCoast(boolean setCoast) {
    // Do nothing if swerve module is already in the requested mode
    if (setCoast == isInCoastMode) return;

    if (setCoast) {
      driveMotor.setNeutralMode(NeutralModeValue.Coast);
      turningMotor.setNeutralMode(NeutralModeValue.Coast);
    } else {
      driveMotor.setNeutralMode(NeutralModeValue.Brake);
      turningMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    isInCoastMode = setCoast;
  }
   
  /**
   * Gets whether the motor is in coast or brake mode.
   * @return true = coast mode, false = brake mode
   */
  public boolean isMotorModeCoast() {
    return isInCoastMode;
  }

  /**
   * Sets the drive motor to FOC or trapezoidal commuatation mode
   * <p><b>NOTE:</b> This takes effect for the <b>next</b> request sent to the motor.
   * @param setFOC true = FOC mode, false = trapezoidal mode
   */
  public void setDriveMotorFOC(boolean setFOC) {
    driveVelocityControl = driveVelocityControl.withEnableFOC(setFOC);
    driveVoltageControl = driveVoltageControl.withEnableFOC(setFOC);
  }


  // ********** Main swerve module control methods

  /**
   * Gets the current state of the module.
   * @return SwerveModuleState state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveEncoderVelocity(), Rotation2d.fromDegrees(getTurningEncoderDegrees()));
  }

  /**
   * Gets the current position of the module.
   * @return SwerveModulePosition position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveEncoderMeters(), Rotation2d.fromDegrees(getTurningEncoderDegrees()));
  }

  /**
   * Turns off the drive and turning motors.
   */
  public void stopMotors() {
    setDriveMotorPercentOutput(0.0);
    setTurnMotorPercentOutput(0.0);
    inVelocityMode = true;
    priorDriveVelocity = 0;
  }

  /**
   * Sets the percent output to the drive motor.
   * @param percentOutput percent output, -1.0 to +1.0
   */
  public void setDriveMotorPercentOutput(double percentOutput) {
    setDriveMotorVoltageOutput(percentOutput * SwerveConstants.voltageCompSaturation);
  }

  /**
   * Sets the voltage output to the drive motor.
   * @param voltage voltage output, nominally -12V to +12V
   */
  public void setDriveMotorVoltageOutput(double voltage) {
    driveMotor.setControl(driveVoltageControl.withOutput(voltage));

    if (voltage == 0.0) {
      inVelocityMode = true;
      priorDriveVelocity = 0;  
    } else {
      inVelocityMode = false;
    }
  }
  
  /**
   * Sets the percent output to the turning motor.
   * @param percentOutput percent output, -1.0 to +1.0
   */
  public void setTurnMotorPercentOutput(double percentOutput) {
    turningMotor.setControl(turningVoltageControl.withOutput(percentOutput * SwerveConstants.voltageCompSaturation));
  }
  
  /**
   * Sets the facing of the wheel (direction the wheel is pointing).
   * <p><b>NOTE:</b> This sets the absolute facing. It does <b>not</b> turn the wheel to the "optimized" 
   * facing +/-180 degrees if that is closer. 
   * @param angle desired wheel facing in degrees, -180 to +180 (positive = left, negative = right, 0 = facing front of robot)
   */
  public void setWheelFacing(double angle){
    angle = MathBCR.normalizeAngle(angle);
    turningMotor.setControl(turningPositionControl.withPosition(calculateTurningEncoderTargetRaw(angle)));
  }

  /**
   * Sets the desired state for the module, using closed loop controls on the Talons.
   * The Talons will hold this state until commanded to stop or another state.
   * @param desiredState desired SwerveModuleState with speed and angle
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
    desiredState = MathSwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getTurningEncoderDegrees()));

    double ff;
    if (inVelocityMode && Math.abs(priorDriveVelocity - desiredState.speedMetersPerSecond) >= SwerveConstants.velMinDeltaUsingkA) {
      // When in velocity mode (from prior call) and not close to desired speed, use kA
      ff = driveFeedforward.calculateWithVelocities(priorDriveVelocity, desiredState.speedMetersPerSecond);
    } else {
      // If actual speed is close to desired speed, then do not use kA (feedback with actual velocity) 
      // in order to prevent velocity oscillation (jitter when robot is stopped or slow).
      // The velocity control kP will provide sufficient feedback to reach the right velocity.
      ff = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
    }

    // Set drive motor velocity or percent output
    if (isOpenLoop) {
      setDriveMotorVoltageOutput(ff);
    } else {
      driveMotor.setControl(driveVelocityControl
        .withVelocity(calculateDriveEncoderVelocityRaw(desiredState.speedMetersPerSecond))
        .withFeedForward(ff));
    }

    // Set turning motor target angle
    turningMotor.setControl(turningPositionControl.withPosition(calculateTurningEncoderTargetRaw(desiredState.angle.getDegrees())));

    // Save state for next call
    inVelocityMode = true;
    priorDriveVelocity = desiredState.speedMetersPerSecond;
  }

  // ********** Encoder methods

  // ******* Drive encoder methods

  /**
   * Gets the encoder position of the drive motor.
   * @return encoder position, in pinion rotations
   */
  public double getDriveEncoderRotations() {
    driveEncoderPostion.refresh();
    return driveEncoderPostion.getValueAsDouble();
  }

  /**
	 * Sets the drive encoder position to zero in software.
	 */
  public void zeroDriveEncoder() {
    driveEncoderZero = getDriveEncoderRotations();
    DataLogUtil.writeMessageEcho(buildString("SwerveModule ", swName), ": ZeroDriveEncoder, driveEncoderZero =", driveEncoderZero, 
    ", raw encoder =", getDriveEncoderRotations(), ", encoder meters =", getDriveEncoderMeters());
  }

  /**
   * Gets the distance traveled by the drive wheel.
   * @return distance traveled, in meters (positive = forward)
   */
  public double getDriveEncoderMeters() {
    return (getDriveEncoderRotations() - driveEncoderZero) * SwerveConstants.kDriveEncoderMetersPerTick;
  }

  /**
   * Gets the velocity of the drive motor.
   * @return velocity, in meters per second (positive = forward)
   */
  public double getDriveEncoderVelocity() {
    driveEncoderVelocity.refresh();
    return driveEncoderVelocity.getValueAsDouble() * SwerveConstants.kDriveEncoderMetersPerTick;
  }

  /**
   * Converts a target velocity (in meters per second) to a target raw drive motor velocity.
   * @param velocityMPS Desired drive wheel velocity, in meters per second (positive = forward)
   * @return drive motor raw value equivalent velocity, in encoder ticks (= pinion rotations) per second
   */
  public double calculateDriveEncoderVelocityRaw(double velocityMPS) {
    return velocityMPS / SwerveConstants.kDriveEncoderMetersPerTick;
  }
  
  // ******* Turning TalonFX encoder methods

  /**
   * Gets the raw encoder position of the turning motor.
   * @return encoder position, in pinion rotations
   */
  public double getTurningEncoderRaw() {
    turningEncoderPosition.refresh();
    return turningEncoderPosition.getValueAsDouble();
  }
  
  /**
   * Calibrates the turning motor encoder. Sets the current wheel facing to currentAngleDegrees.
   * @param currentAngleDegrees current angle, in degrees
   */
  public void calibrateTurningEncoderDegrees(double currentAngleDegrees) {
    turningEncoderZero = getTurningEncoderRaw() - (currentAngleDegrees / SwerveConstants.kTurningEncoderDegreesPerTick);
    DataLogUtil.writeMessageEcho(buildString("SwerveModule ", swName), ": calibrateTurningEncoder, turningEncoderZero =", turningEncoderZero, 
    ", raw encoder =", getTurningEncoderRaw(), ", set degrees =", currentAngleDegrees, ", encoder degrees =", getTurningEncoderDegrees());
  }

  /**
   * Gets the turning motor encoder facing. When calibrated, 0 should be with the wheel pointing toward 
   * the front of robot (positive = counterclockwise, negative = clockwise).
   * @return encoder facing, in degrees (does not wrap, so could be greater than 360 degrees)
   */
  public double getTurningEncoderDegrees() {
    return (getTurningEncoderRaw() - turningEncoderZero) * SwerveConstants.kTurningEncoderDegreesPerTick;
  }

  /**
   * Converts a target wheel facing (in degrees) to a target raw turning motor encoder value.
   * @param targetDegrees desired wheel facing, in degrees (0 = towards front of robot, positive = counterclockwise, negative = clockwise)
   * @return turning motor encoder raw value equivalent to input facing
   */
  public double calculateTurningEncoderTargetRaw(double targetDegrees) {
    return targetDegrees / SwerveConstants.kTurningEncoderDegreesPerTick + turningEncoderZero;
  }

  /**
   * Gest the rotational velocity of the turning encoder.
   * @return turning TalonFX encoder rotational velocity for wheel facing, in degrees/second (positive = counterclockwise, negative = clockwise)
   */
  public double getTurningEncoderVelocityDPS() {
    turningEncoderVelocity.refresh();
    return turningEncoderVelocity.getValueAsDouble() * SwerveConstants.kTurningEncoderDegreesPerTick;
  }

  // ******* Cancoder methods

  /**
   * Calibrates the CANcoder encoder so that 0 should be with the wheel pointing toward the front of robot.
   * @param offsetDegrees desired encoder zero point, in absolute magnet position reading
   */
  public void calibrateCanCoderDegrees(double offsetDegrees) {
    // System.out.println(swName + " " + turningOffsetDegrees);
    // turningCanCoder.configMagnetOffset(offsetDegrees, 100);
    cancoderZero = -offsetDegrees;
    DataLogUtil.writeMessageEcho(buildString("SwerveModule ", swName), ": calibrateCanCoder, cancoderZero =", cancoderZero, 
    ", raw encoder =", turningCanCoderPosition.refresh().getValueAsDouble() * 360.0, ", encoder degrees =", getCanCoderDegrees());
  }

  /**
   * Gets the wheel facing of the turning motor. When calibrated, 0 should be with the wheel pointing toward 
   * the front of robot (positive = counterclockwise, negative = clockwise).
   * @return wheel facing, in degrees [-180,+180)
   */
  public double getCanCoderDegrees() {
    turningCanCoderPosition.refresh();
    return MathBCR.normalizeAngle(turningCanCoderPosition.getValueAsDouble() * 360.0 - cancoderZero);
  }

  /**
   * Gets the rotatioal velocity of the turning CANcoder.
   * @return rotational velocity for wheel facing, in degrees/second (positive = counterclockwise, negative = clockwise)
   */
  public double getCanCoderVelocityDPS() {
    turningCanCoderVelocity.refresh();
    return turningCanCoderVelocity.getValueAsDouble() * 360.0;
  }


  // ********** Information methods

  public double getDriveBusVoltage() {
    driveMotorSupplyVoltage.refresh();
    return driveMotorSupplyVoltage.getValueAsDouble();
  }

  public double getDriveOutputPercent() {
    driveDutyCycle.refresh();
    return driveDutyCycle.getValueAsDouble();
  }

  public double getDriveStatorCurrent() {
    driveStatorCurrent.refresh();
    return driveStatorCurrent.getValueAsDouble();
  }

  public double getDriveTemp() {
    driveMotorTemp.refresh();
    return driveMotorTemp.getValueAsDouble();
  }

  public double getTurningOutputPercent() {
    turningDutyCycle.refresh();
    return turningDutyCycle.getValueAsDouble();
  }

  public double getTurningStatorCurrent() {
    turningStatorCurrent.refresh();
    return turningStatorCurrent.getValueAsDouble();
  }

  public double getTurningTemp() {
    turningMotorTemp.refresh();
    return turningMotorTemp.getValueAsDouble();
  }

  /**
   * Returns true if there is a hardware error, and logs the hardware error to a
   * Sticky Fault, to the log file, and to the console.  Multiple errors result
   * in a concatenation of values.
   * <p> "Drive" = drive motor not connected to CANbus
   * <p> "Turn" = turning motor not connected to CANbus
   * <p> "Cancoder" = turning Cancoder not connected to CANbus
   * @return false = no hardware errors, true = one or more hardware errors
   */
  public boolean checkForFaultConditions() {
    String s = "";
    if (!driveMotor.isConnected()) s = StringUtil.buildString(s, "-Drive");
    if (!turningMotor.isConnected()) s = StringUtil.buildString(s, "-Turn");
    if (!turningCanCoder.isConnected()) s = StringUtil.buildString(s, "-Cancoder");

    if (s.isEmpty()) {
      return false;
    }

    s = StringUtil.buildString("SweveModule-", swName, s);
    RobotPreferences.recordStickyFaults(s);
    DataLogUtil.writeMessageEcho(StringUtil.buildString("STICKY FAULT ", s));

    return true;
  }
  
  /**
   * Updates relevant variables on Elastic.
   */
  public void updateElastic() {
    SmartDashboard.putNumber(buildString("Swerve FXangle ", swName), MathBCR.normalizeAngle(getTurningEncoderDegrees()));
    SmartDashboard.putNumber(buildString("Swerve CCangle ", swName), getCanCoderDegrees());
    SmartDashboard.putNumber(buildString("Swerve FXangle dps", swName), getTurningEncoderVelocityDPS());
    SmartDashboard.putNumber(buildString("Swerve distance", swName), getDriveEncoderMeters());
    SmartDashboard.putNumber(buildString("Swerve drive temp ", swName), getDriveTemp());
  }

  /**
   * Writes swerve module data to the logfile
   * @param timeNow Timestamp for logging data
   */
  public void updateSwerveLog(long timeNow) {
    dLogCCAngle.append(getCanCoderDegrees(), timeNow);
    dLogCCTurnRate.append(getCanCoderVelocityDPS(), timeNow);
    dLogFXAngle.append(MathBCR.normalizeAngle(getTurningEncoderDegrees()), timeNow);
    dLogFXTurnRate.append(getTurningEncoderVelocityDPS(), timeNow);
    dLogTurnTemp.append(getTurningTemp(), timeNow);
    dLogTurnOutput.append(getTurningOutputPercent(), timeNow);
    dLogDriveTemp.append(getDriveTemp(), timeNow);
    dLogDriveOutput.append(getDriveOutputPercent(), timeNow);
    dLogDriveDist.append(getDriveEncoderMeters(), timeNow);
    dLogDriveSpeed.append(getDriveEncoderVelocity(), timeNow);
    dLogDriveCurrent.append(getDriveStatorCurrent(), timeNow);
  }
}
