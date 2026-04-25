// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeDeployAngle;
import frc.robot.Constants.Ports;
import frc.robot.utilities.DataLogUtil;
// import frc.robot.utilities.LEDEventUtil;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.Wait;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class Intake extends SubsystemBase implements Loggable{
  private int logRotationKey;           // Key to identify the log rotation
  private boolean fastLogging = false;  // true = enabled to run every cycle, false = follows normal logging cycles
  private String subsystemName;         // Used in file logging and SmartDashboard for naming log entries

  // Motors and CANcoder
  private final TalonFX intakeRollerMotor = new TalonFX(Ports.CANIntakeRollerMotor);
  private final TalonFX intakeDeployMotor = new TalonFX(Ports.CANIntakeDeployMotor);
  private final CANcoder intakeDeployCANcoder = new CANcoder(Ports.CANIntakeDeployEncoder);

  // Configurators
  private final TalonFXConfigurator intakeRollerConfigurator = intakeRollerMotor.getConfigurator();
  private final TalonFXConfigurator intakeDeployConfigurator = intakeDeployMotor.getConfigurator();
  private final CANcoderConfigurator intakeDeployCANcoderConfigurator = intakeDeployCANcoder.getConfigurator();

  // Configurations
  private TalonFXConfiguration intakeRollerMotorConfig;
  private TalonFXConfiguration intakeDeployMotorCANcoderConfig;
  private TalonFXConfiguration intakeDeployMotorRotorConfig;
  private CANcoderConfiguration intakeDeployCANcoderConfig;

  // Status signals for the roller motor
  private final StatusSignal<Temperature> intakeRollerTemp;                 // Motor temperature, in degrees Celsius
  private final StatusSignal<Current> intakeRollerStatorCurrent;            // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Voltage> intakeRollerVoltage;                  // Motor voltage, in volts
  private final StatusSignal<AngularVelocity> intakeRollerEncoderVelocity;  // Motor velocity measured by the encoder, in rotations/second

  // Status signals for the deploy motor
  private final StatusSignal<Temperature> intakeDeployTemp;                 // Motor temperature, in degrees Celsius
  private final StatusSignal<Current> intakeDeployStatorCurrent;            // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Voltage> intakeDeployVoltage;                  // Motor voltage, in volts
  private final StatusSignal<AngularVelocity> intakeDeployEncoderVelocity;  // Motor velocity measured by the encoder, in rotations/second
  private final StatusSignal<Angle> intakeDeployEncoderPosition;            // Motor position measured by the encoder, in rotations

  // Status signals for the deploy motor CANcoder
  private final StatusSignal<MagnetHealthValue> intakeDeployCANcoderMagnetHealth; // CANcoder magnet health
  private final StatusSignal<Angle> intakeDeployCANcoderPosition;                 // CANcoder position, in CANcoder rotations

  // Motor control modes
  private VoltageOut intakeRollerVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private VoltageOut intakeDeployVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private MotionMagicVoltage intakeDeployMMVoltageControl = new MotionMagicVoltage(0.0).withEnableFOC(false);

  // DataLog
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogRollerTemp = new DoubleLogEntry(log, "/IntakeRoller/Temperature");
  private final DoubleLogEntry dLogRollerCurrent = new DoubleLogEntry(log, "/IntakeRoller/Current");
  private final DoubleLogEntry dLogRollerVelocity = new DoubleLogEntry(log, "/IntakeRoller/Velocity");
  private final DoubleLogEntry dLogRollerVoltage = new DoubleLogEntry(log, "/IntakeRoller/Voltage");
  private final DoubleLogEntry dLogDeployTemp = new DoubleLogEntry(log, "/IntakeDeploy/Temperature");
  private final DoubleLogEntry dLogDeployCurrent = new DoubleLogEntry(log, "/IntakeDeploy/Current");
  private final DoubleLogEntry dLogDeployVelocity = new DoubleLogEntry(log, "/IntakeDeploy/VelocityRPS");
  private final DoubleLogEntry dLogDeployVoltage = new DoubleLogEntry(log, "/IntakeDeploy/Voltage");
  private final DoubleLogEntry dLogDeployPosition = new DoubleLogEntry(log, "/IntakeDeploy/Position");
  private final DoubleLogEntry dLogDeployTargetPosition = new DoubleLogEntry(log, "/IntakeDeploy/TargetPosition");

  private boolean isIntakeDeployCalibrated = false; // true = deploy motor is calibrated, false = not calibrated
  private boolean isIntakeDeployUsingCANcoder;      // true = deploy motor is using CANcoder, false = using rotor encoder
  private boolean isIntakeDeployInPositionControl;  // true = deploy motor is in position control, false = not position control

  private double intakeDeployEncoderZero = 0;       // Encoder reading when the deploy motor angle is zero
  private double intakeDeploySafeSetpointAngle;     // Current setpoint angle for the deploy motor (position control), constrained by calibration and interlocks
  private double intakeDeploySetpointPercent;       // Current setpoint percent output for the deploy motor
  private double intakeRollerSetpointPercent;       // Current setpoint percent output for the roller motor

  public Intake(String subsystemName) {
    this.subsystemName = subsystemName;
    logRotationKey = DataLogUtil.allocateLogRotation();

    // Get status signals for roller motor
    intakeRollerTemp = intakeRollerMotor.getDeviceTemp();
    intakeRollerStatorCurrent = intakeRollerMotor.getStatorCurrent();
    intakeRollerEncoderVelocity = intakeRollerMotor.getVelocity();
    intakeRollerVoltage = intakeRollerMotor.getMotorVoltage();
  
    // Get status signals for deploy motor
    intakeDeployTemp = intakeDeployMotor.getDeviceTemp();
    intakeDeployStatorCurrent = intakeDeployMotor.getStatorCurrent();
    intakeDeployEncoderVelocity = intakeDeployMotor.getVelocity();
    intakeDeployVoltage = intakeDeployMotor.getMotorVoltage();
    intakeDeployEncoderPosition = intakeDeployMotor.getPosition();

    // Get status signals for deploy motor CANcoder
    intakeDeployCANcoderMagnetHealth = intakeDeployCANcoder.getMagnetHealth();
    intakeDeployCANcoderPosition = intakeDeployCANcoder.getPosition();

    // Configure the roller motor
    intakeRollerMotorConfig = new TalonFXConfiguration();
    intakeRollerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeRollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeRollerMotorConfig.Voltage.PeakForwardVoltage = IntakeConstants.rollerCompensationVoltage;
    intakeRollerMotorConfig.Voltage.PeakReverseVoltage = -IntakeConstants.rollerCompensationVoltage;
    intakeRollerMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;      // Time from 0 to full power, in seconds
    intakeRollerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3;  // Time from 0 to full power, in seconds

    // Configure the deploy motor when using the rotor encoder
    intakeDeployMotorRotorConfig = new TalonFXConfiguration();
    intakeDeployMotorRotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeDeployMotorRotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeDeployMotorRotorConfig.Voltage.PeakForwardVoltage = IntakeConstants.deployCompensationVoltage;
    intakeDeployMotorRotorConfig.Voltage.PeakReverseVoltage = -IntakeConstants.deployCompensationVoltage;
    intakeDeployMotorRotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;      // Time from 0 to full power, in seconds
    intakeDeployMotorRotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;  // Time from 0 to full power, in seconds

    // Configure the deploy motor when using the CANcoder
    intakeDeployMotorCANcoderConfig = new TalonFXConfiguration();
    intakeDeployMotorCANcoderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeDeployMotorCANcoderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeDeployMotorCANcoderConfig.Voltage.PeakForwardVoltage = IntakeConstants.deployCompensationVoltage;
    intakeDeployMotorCANcoderConfig.Voltage.PeakReverseVoltage = -IntakeConstants.deployCompensationVoltage;
    intakeDeployMotorCANcoderConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;      // Time from 0 to full power, in seconds
    intakeDeployMotorCANcoderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;  // Time from 0 to full power, in seconds

    // Prevent the breakers from tripping by limiting current if above the limit for the threshold time
    intakeRollerMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;           // Upper limit for the current, in amps
    intakeRollerMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;      // Lower limit for the current, in amps
    intakeRollerMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;        // Threshold time, in seconds
    intakeRollerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;     // Enable the limit
    intakeDeployMotorRotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;        // Upper limit for the current, in amps
    intakeDeployMotorRotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;   // Lower limit for the current, in amps
    intakeDeployMotorRotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;     // Threshold time, in seconds
    intakeDeployMotorRotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;  // Enable the limit
    intakeDeployMotorCANcoderConfig.CurrentLimits.SupplyCurrentLimit = 60.0;        // Upper limit for the current, in amps
    intakeDeployMotorCANcoderConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;   // Lower limit for the current, in amps
    intakeDeployMotorCANcoderConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;     // Threshold time, in seconds
    intakeDeployMotorCANcoderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;  // Enable the limit

    // Configure the deploy motor CANcoder
    intakeDeployCANcoderConfig = new CANcoderConfiguration();
    intakeDeployCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    intakeDeployCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = IntakeConstants.deployCANcoderDiscontinuityPoint;
    intakeDeployCANcoderConfig.MagnetSensor.MagnetOffset = 0.0;

    // Turn off soft limits until after calibration
    intakeDeployMotorCANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    intakeDeployMotorCANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // Configure Magic Motion for position control with the deploy motor
    intakeDeployMotorRotorConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.MMCruiseVelocity;
    intakeDeployMotorRotorConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.MMAcceleration;
    intakeDeployMotorRotorConfig.MotionMagic.MotionMagicJerk = IntakeConstants.MMJerk;
    intakeDeployMotorCANcoderConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.MMCruiseVelocity;
    intakeDeployMotorCANcoderConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.MMAcceleration;
    intakeDeployMotorCANcoderConfig.MotionMagic.MotionMagicJerk = IntakeConstants.MMJerk;

    // Configure rotor encoder as a feedback source for the deploy motor
    // When using the rotor encoder, RotorToSensorRatio = 1.0 and SensorToMechanismRatio = gear ratio
    intakeDeployMotorRotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    intakeDeployMotorRotorConfig.Feedback.RotorToSensorRatio = 1.0;
    intakeDeployMotorRotorConfig.Feedback.SensorToMechanismRatio = IntakeConstants.deployGearRatio;
    intakeDeployMotorRotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    
    // Assign a slot for the deploy motor position control
    intakeDeployMMVoltageControl.Slot = 0;
    intakeDeployMMVoltageControl.OverrideBrakeDurNeutral = true;

    intakeDeployMotorRotorConfig.Slot0.kP = IntakeConstants.kP;
    intakeDeployMotorRotorConfig.Slot0.kI = IntakeConstants.kI;
    intakeDeployMotorRotorConfig.Slot0.kD = IntakeConstants.kD;
    intakeDeployMotorRotorConfig.Slot0.kG = 0.0;
    intakeDeployMotorRotorConfig.Slot0.kS = IntakeConstants.kS;
    intakeDeployMotorRotorConfig.Slot0.kV = IntakeConstants.kV;
    intakeDeployMotorRotorConfig.Slot0.kA = IntakeConstants.kA;

    // Configure the CANcoder as a feedback source for the deploy motor
    // When using the CANcoder, SensorToMechanismRatio = 1.0 and RotorToSensorRatio = gear ratio
    intakeDeployMotorCANcoderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    intakeDeployMotorCANcoderConfig.Feedback.FeedbackRemoteSensorID = Ports.CANIntakeDeployEncoder;
    intakeDeployMotorCANcoderConfig.Feedback.SensorToMechanismRatio = IntakeConstants.deploySensorToMechanismRatio; // 18 : 36 gear ratio on competition robot (reduction)
    intakeDeployMotorCANcoderConfig.Feedback.RotorToSensorRatio = IntakeConstants.deployGearRatio * (1.0 / IntakeConstants.deploySensorToMechanismRatio);
    intakeDeployMotorCANcoderConfig.ClosedLoopGeneral.ContinuousWrap = false;

    intakeDeployMotorCANcoderConfig.Slot0.kP = IntakeConstants.kP;
    intakeDeployMotorCANcoderConfig.Slot0.kI = IntakeConstants.kI;
    intakeDeployMotorCANcoderConfig.Slot0.kD = IntakeConstants.kD;
    intakeDeployMotorCANcoderConfig.Slot0.kG = 0.0;
    intakeDeployMotorCANcoderConfig.Slot0.kS = IntakeConstants.kS;
    intakeDeployMotorCANcoderConfig.Slot0.kV = IntakeConstants.kV;
    intakeDeployMotorCANcoderConfig.Slot0.kA = IntakeConstants.kA;

    // Apply the configurations to the roller motor. This is a blocking call and will wait up to 200ms to apply.
    intakeRollerConfigurator.apply(intakeRollerMotorConfig);
    Wait.waitTime(250);

    // Apply the configuration to the deploy motor CANcoder. This is a blocking call and will wait up to 200ms to apply.
    intakeDeployCANcoderConfigurator.apply(intakeDeployCANcoderConfig);
    Wait.waitTime(250);

    // Check if the deploy motor CANcoder is connected
    isIntakeDeployUsingCANcoder = isIntakeDeployCANcoderConnected();

    // If the CANcoder is connected, use it for deploy motor position control
    if (isIntakeDeployUsingCANcoder) {
      intakeDeployEncoderZero = IntakeConstants.deployCANcoderOffsetAngle;
      isIntakeDeployCalibrated = true;

      DataLogUtil.writeMessageEcho(
        subsystemName,
        ": CANcoder calibrated",
        ", Enc Zero =", intakeDeployEncoderZero,
        ", CANcoder Rot =", getIntakeDeployCANcoderRotationsRaw(), 
        ", Enc Raw =", getIntakeDeployRotorEncoderRotationsRaw(),
        ", Angle =", getIntakeDeployAngle(),
        ", Target =", getIntakeDeployTargetAngle()
      );

      // Set software limits
      intakeDeployMotorCANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = convertIntakeDeployDegreesToEncoderRotations(IntakeDeployAngle.UPPER_LIMIT.value);
      intakeDeployMotorCANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = convertIntakeDeployDegreesToEncoderRotations(IntakeDeployAngle.LOWER_LIMIT.value);
      intakeDeployMotorCANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      intakeDeployMotorCANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

      // Apply the configurations to the deploy motor. This is a blocking call and will wait up to 200ms to apply.
      intakeDeployConfigurator.apply(intakeDeployMotorCANcoderConfig);

      // NOTE: When the TalonFX encoder settings are changed above, then the next call to get the position 
      // may contain an old value, not the value based on the updated configuration settings above. The CANBus runs 
      // asynchronously from this code, so sending the updated configuration to the CANcoder/TalonFX and then receiving 
      // an updated position measurement back may take longer than this code. The timeouts in the configuration code 
      // above should take care of this, but it does not always wait long enough. Therefore, we add a wait here.
      Wait.waitTime(250);
    } else {
      // Mark the intake as uncalibrated and record a sticky fault
      isIntakeDeployCalibrated = false;
      RobotPreferences.recordStickyFaults("Intake-CANcoder");
      DataLogUtil.writeMessageEcho("Intake: CANcoder not calibrated");

      // Set software limits
      intakeDeployMotorRotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = convertIntakeDeployDegreesToEncoderRotations(IntakeDeployAngle.UPPER_LIMIT.value);
      intakeDeployMotorRotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = convertIntakeDeployDegreesToEncoderRotations(IntakeDeployAngle.LOWER_LIMIT.value);
      intakeDeployMotorRotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      intakeDeployMotorRotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

      
      // Apply the configurations to the deploy motor.
      // This is a blocking call and will wait up to 200ms to apply.
      intakeDeployConfigurator.apply(intakeDeployMotorRotorConfig);

      // NOTE: When the TalonFX encoder settings are changed above, then the next call to get the position 
      // may contain an old value, not the value based on the updated configuration settings above. The CANBus runs 
      // asynchronously from this code, so sending the updated configuration to the CANcoder/TalonFX and then receiving 
      // an updated position measurement back may take longer than this code. The timeouts in the configuration code 
      // above should take care of this, but it does not always wait long enough. Therefore, we add a wait here.
      Wait.waitTime(250);
    }

    // Stop motors
    stopIntakeRoller();
    stopIntakeDeploy();

    // Prime the DataLog to reduce delay when first enabling the robot
    updateLog(true);
  }

  /**
   * Sets the percent output of the intakeRollerMotor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = intake, negative = outtake)
   */
  public void setIntakeRollerPercentOutput(double percent) {
    intakeRollerMotor.setControl(intakeRollerVoltageControl.withOutput(percent * IntakeConstants.rollerCompensationVoltage));
    intakeRollerSetpointPercent = percent;
  
    // Send an LED event based on whether the rollers are stopped/intaking/outtaking
    // if (percent == 0.0) LEDEventUtil.sendEvent(LEDEventUtil.LEDStripEvents.NEUTRAL);
    // else if (percent > 0.0) LEDEventUtil.sendEvent(LEDEventUtil.LEDStripEvents.INTAKING);
    // else LEDEventUtil.sendEvent(LEDEventUtil.LEDStripEvents.REVERSE_INTAKING);
  }

  /**
   * Sets the percent output of the intakeDeployMotor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = deploy, negative = retract)
   */
  public void setIntakeDeployPercentOutput(double percent) {
    isIntakeDeployInPositionControl = false;
    intakeDeployMotor.setControl(intakeDeployVoltageControl.withOutput(percent * IntakeConstants.rollerCompensationVoltage));
    intakeDeploySetpointPercent = percent;
  }

  /**
   * Stops the intakeRollerMotor.
   */
  public void stopIntakeRoller() {
    setIntakeRollerPercentOutput(0);
  }

  /**
   * Stops the intakeDeployMotor.
   */
  public void stopIntakeDeploy() {
    setIntakeDeployPercentOutput(0);
  }

  /**
   * Sets the angle of the intakeDeployMotor encoder, constrained by calibration and interlocks.
   * @param angle target angle, in degrees (0 = front plate level with bubble, positive = in, negative = out)
   */
  public void setIntakeDeployAngle(double angle) {
    if (!isIntakeDeployCalibrated) {
      DataLogUtil.writeMessageEcho(
        subsystemName,
        ": setIntakeDeployAngle",
        ", Desired Angle =", angle,
        ", Aborted (uncalibrated)"
      );
      return;
    }

    isIntakeDeployInPositionControl = true;

    // Keep the position in a usable range
    intakeDeploySafeSetpointAngle = MathUtil.clamp(angle, IntakeDeployAngle.LOWER_LIMIT.value, IntakeDeployAngle.UPPER_LIMIT.value);

    // Set the position by converting degrees to encoder rotations and modifying feed foward
    intakeDeployMotor.setControl(intakeDeployMMVoltageControl
      .withPosition(convertIntakeDeployDegreesToEncoderRotations(intakeDeploySafeSetpointAngle))
      .withFeedForward(-IntakeConstants.kG * Math.sin(intakeDeploySafeSetpointAngle * Math.PI / 180.0))
    );

    DataLogUtil.writeMessageEcho(
      subsystemName,
      ": setIntakeDeployAngle",
      ", Desired Angle =", angle,
      ", Set Angle =", intakeDeploySafeSetpointAngle
    );
  }

  /**
   * Gets the current setpoint percent output for the intakeRollerMotor.
   * @return setpoint percent, -1.0 to 1.0
   */
  public double getIntakeRollerSetpointPercent() {
    return intakeRollerSetpointPercent;
  }

  /**
   * Gets the current setpoint percent output for the intakeDeployMotor.
   * @return setpoint percent, -1.0 to 1.0
   */
  public double getIntakeDeploySetpointPercent() {
    return intakeDeploySetpointPercent;
  }

  /**
   * Gets the raw number of rotations from the CANcoder.
   * @return raw encoder reading, in CANcoder rotations
   */
  public double getIntakeDeployCANcoderRotationsRaw() {
    intakeDeployCANcoderPosition.refresh(); // Verified that this is not a blocking call
    return intakeDeployCANcoderPosition.getValueAsDouble();
  }

  /**
   * Gets the raw number of rotations from the rotor encoder.
   * @return raw encoder reading, in pinion rotations
   */
  public double getIntakeDeployRotorEncoderRotationsRaw() {
    intakeDeployEncoderPosition.refresh(); // Verified that this is not a blocking call
    return intakeDeployEncoderPosition.getValueAsDouble();
  }

  /**
   * Gets the angle that the intakeDeployMotor is trying to move to. If uncalibrated, this value is the lower limit,
   * since we do not know the current position. If not in position control, this is the actual position.
   * @returns target angle, in degrees
   */
  public double getIntakeDeployTargetAngle() {
    // If uncalibrated, we do not know the current position, so return the lower limit
    if (!isIntakeDeployCalibrated) return IntakeDeployAngle.LOWER_LIMIT.value;

    // If using position control, return the safe angle for deployment
    if (isIntakeDeployMotorUsingPositionControl()) return intakeDeploySafeSetpointAngle;

    // Otherwise, return the current angle
    return getIntakeDeployAngle();
  }

  /**
   * Gets whether the intakeDeployMotor is using position control.
   * @return true = position control, false = not position control (e.g. voltage control)
   */
  public boolean isIntakeDeployMotorUsingPositionControl() {
    return isIntakeDeployInPositionControl;
  }

  /**
   * Gets whether the CANcoder is connected and its health is ideal (green) or adequate (orange).
   * @return true = CANcoder is connected, false = CANcoder is not connected
   */
  public boolean isIntakeDeployCANcoderConnected() {
    intakeDeployCANcoderMagnetHealth.refresh();
    MagnetHealthValue magnetHealth = intakeDeployCANcoderMagnetHealth.getValue();
    return intakeDeployCANcoder.isConnected() && (magnetHealth == MagnetHealthValue.Magnet_Green || magnetHealth == MagnetHealthValue.Magnet_Orange);
  }

  /**
   * Returns if the intake deploy is within safe angles for going under the trench
   * @return true = intake deploy is within safe angles, false = intake deploy isoutside of safe angles
   */
  public boolean isIntakeDeploySafeForTrench() {
    return getIntakeDeployAngle() < IntakeConstants.IntakeDeployAngle.TRENCH_ALLOWED_LOWER.value || 
    getIntakeDeployAngle() > IntakeConstants.IntakeDeployAngle.TRENCH_ALLOWED_UPPER.value;
  }

  /**
   * Gets the velocity of the intakeRollerMotor.
   * @return motor velocity, in RPM
   */
  public double getIntakeRollerVelocity() {
    intakeRollerEncoderVelocity.refresh();
    return intakeRollerEncoderVelocity.getValueAsDouble() * 60.0;
  }

  /**
   * Gets the current of the intakeRollerMotor.
   * @return motor current, in amps
   */
  public double getIntakeRollerAmps() {
    intakeRollerStatorCurrent.refresh();
    return intakeRollerStatorCurrent.getValueAsDouble();
  }

  /**
   * Gets the velocity of the intakeDeployMotor.
   * @return motor velocity, in RPS
   */
  public double getIntakeDeployVelocity() {
    intakeDeployEncoderVelocity.refresh();
    return intakeDeployEncoderVelocity.getValueAsDouble();
  }

  /**
   * Gets the current of the intakeDeployMotor.
   * @return motor current, in amps
   */
  public double getIntakeDeployAmps() {
    intakeDeployStatorCurrent.refresh();
    return intakeDeployStatorCurrent.getValueAsDouble();
  }

  /**
   * Gets the current angle of the intakeDeployMotor from the rotor encoder, assuming it is calibrated.
   * NOTE: This is for internal subsystem use only. Use getIntakeDeployAngle() externally.
   * @return current rotor encoder angle, in degrees (based on raw encoder ticks offset by the encoder zero)
   */
  private double getIntakeDeployRotorEncoderDegrees() {
    // NOTE: Do not normalize this angle. It should not wrap, since the mechanism cannot cross the -180/+180 degree point.
    return getIntakeDeployRotorEncoderRotationsRaw() * IntakeConstants.deployDegreesPerRotation - intakeDeployEncoderZero;
  }

  /**
   * Gets the current angle of the intakeDeployMotor. If uncalibrated, this value is the lower limit,
   * since we do not know the current position. Otherwise, this is the actual position.
   * @returns current angle, in degrees
   */
  public double getIntakeDeployAngle() {
    if (!isIntakeDeployCalibrated) return IntakeDeployAngle.LOWER_LIMIT.value;
    return getIntakeDeployRotorEncoderDegrees();
  }

  /**
   * Converts the intakeDeployMotor angle from degrees to rotations, assuming it is calibrated.
   * @param degrees angle, in degrees
   * @return angle, in rotations
   */
  private double convertIntakeDeployDegreesToEncoderRotations(double degrees) {
    return (degrees + intakeDeployEncoderZero) / IntakeConstants.deployDegreesPerRotation;
  }

  /**
   * Gets whether the intakeDeployMotor is calibrated for position control.
   * @return true = intakeDeployMotor is calibrated for position control, false = not calibrated
   */
  public boolean isIntakeDeployCalibrated() {
    return isIntakeDeployCalibrated;
  }  

  /**
   * Calibrates the intakeDeployMotor with a known current angle.
   * @param angle current physical angle of the intakeDeployMotor, in degrees
   */
  public void calibrateIntakeDeployEncoder(double angle) {
    // Stop the motor so it does not jump to a new value
    stopIntakeDeploy();

    // Recalculate the encoder zero based on the current position and input angle
    intakeDeployEncoderZero = getIntakeDeployRotorEncoderRotationsRaw() * IntakeConstants.deployDegreesPerRotation - angle;

    // Mark as calibrated
    isIntakeDeployCalibrated = true;

    DataLogUtil.writeMessageEcho(
      subsystemName,
      ": Calibrate",
      ", Using CANcoder", isIntakeDeployUsingCANcoder,
      ", Enc Zero =", intakeDeployEncoderZero,
      ", CANcoder Rot =", getIntakeDeployCANcoderRotationsRaw(),
      ", Enc Raw =", getIntakeDeployRotorEncoderRotationsRaw(),
      ", Angle =", getIntakeDeployAngle(),
      ", Target =", getIntakeDeployTargetAngle()
    );

    if (isIntakeDeployUsingCANcoder) {
      // Set software limits after updating encoder zero
      intakeDeployMotorCANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = convertIntakeDeployDegreesToEncoderRotations(IntakeDeployAngle.UPPER_LIMIT.value);
      intakeDeployMotorCANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = convertIntakeDeployDegreesToEncoderRotations(IntakeDeployAngle.LOWER_LIMIT.value);
      intakeDeployMotorCANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      intakeDeployMotorCANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  
      // Apply the configurations. This is a blocking call and will wait up to 200ms to apply.
      intakeDeployConfigurator.apply(intakeDeployMotorCANcoderConfig);      
    } else {
      // Set software limits after updating encoder zero
      intakeDeployMotorRotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = convertIntakeDeployDegreesToEncoderRotations(IntakeDeployAngle.UPPER_LIMIT.value);
      intakeDeployMotorRotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = convertIntakeDeployDegreesToEncoderRotations(IntakeDeployAngle.LOWER_LIMIT.value);
      intakeDeployMotorRotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      intakeDeployMotorRotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  
      // Apply the configurations. This is a blocking call and will wait up to 200ms to apply.
      intakeDeployConfigurator.apply(intakeDeployMotorRotorConfig);      
    }
  }

  /**
   * Gets the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Turns file logging on every scheduler cycle (~20 msec) or every 10 cycles (~0.2 sec).
   * @param enabled true = log every cycle, false = log every 10 cycles
   */
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Writes information about the subsystem to the file log.
   * @param logWhenDisabled true = log always, false = only log when enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    if (!logWhenDisabled && DriverStation.isDisabled()) return;

    long timeNow = RobotController.getFPGATime();

    dLogRollerTemp.append(intakeRollerTemp.refresh().getValueAsDouble(), timeNow);
    dLogRollerVoltage.append(intakeRollerVoltage.refresh().getValueAsDouble(), timeNow);
    dLogRollerCurrent.append(getIntakeRollerAmps(), timeNow);
    dLogRollerVelocity.append(getIntakeRollerVelocity(), timeNow);

    dLogDeployTemp.append(intakeDeployTemp.refresh().getValueAsDouble(), timeNow);
    dLogDeployVoltage.append(intakeDeployVoltage.refresh().getValueAsDouble(), timeNow);
    dLogDeployCurrent.append(getIntakeDeployAmps(), timeNow);
    dLogDeployVelocity.append(getIntakeDeployVelocity(), timeNow);
    dLogDeployPosition.append(getIntakeDeployAngle(), timeNow);
    dLogDeployTargetPosition.append(getIntakeDeployTargetAngle(), timeNow);
  }

  @Override
  public void periodic() {
    double rollerAmps = getIntakeRollerAmps();
    double deployAmps = getIntakeDeployAmps();

    if (rollerAmps > 100) {
      setIntakeRollerPercentOutput(0);
      DataLogUtil.writeMessageEcho(subsystemName, ": IntakeRoller", ", EXCEEDED MAX CURRENT");
    }

    if (deployAmps > 100) {
      setIntakeDeployPercentOutput(0);
      DataLogUtil.writeMessageEcho(subsystemName, ": IntakeDeploy", ", EXCEEDED MAX CURRENT");
    }

    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }

    if (DataLogUtil.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean(INTAKE_DEPLOY_IS_CANCODER_CONNECTED.value, isIntakeDeployCANcoderConnected());
      SmartDashboard.putBoolean(INTAKE_DEPLOY_IS_CALIBRATED.value, isIntakeDeployCalibrated());
      SmartDashboard.putNumber(INTAKE_DEPLOY_TEMP.value, intakeRollerTemp.refresh().getValueAsDouble());    
      SmartDashboard.putNumber(INTAKE_DEPLOY_CANCODER_RAW_ROTATIONS.value, getIntakeDeployCANcoderRotationsRaw());
      SmartDashboard.putNumber(INTAKE_DEPLOY_ANGLE.value, getIntakeDeployAngle());
      SmartDashboard.putNumber(INTAKE_DEPLOY_TARGET_ANGLE.value, getIntakeDeployTargetAngle());

      SmartDashboard.putNumber(INTAKE_ROLLER_TEMP.value, intakeRollerTemp.refresh().getValueAsDouble());
      SmartDashboard.putNumber(INTAKE_ROLLER_CURRENT.value, rollerAmps);
      SmartDashboard.putNumber(INTAKE_ROLLER_RPM.value, getIntakeRollerVelocity());
    }

  }
}
