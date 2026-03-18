// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.Wait;
import frc.robot.Constants.*;
import frc.robot.Constants.TurretConstants.TurretAngle;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class Turret extends SubsystemBase implements Loggable {

  private final int logRotationKey;
  private boolean fastLogging = false; // true = enabled to run every cycle, false = follow normal logging cycles
  private String subsystemName; // Subsystem name for use in file logging and dashboard

  // Create turret motor
  private final TalonFX turretMotor = new TalonFX(Ports.CANTurretMotor);
  private final TalonFXConfigurator turretMotorConfigurator = turretMotor.getConfigurator();

  // We create two configs for the turret motor -- one with RemoteCANcoder and one
  // with RotorEncoder. The default config is RemoteCANcoder. If the CANcoder fails,
  // the turret will use the RotorEncoder config as a fallback.
  private boolean usingEncoder1; // true = using encoder 1 configuration, false = using RotorEncoder configuration
  private TalonFXConfiguration usingRotorEncoderConfig; // Configuration for reading the rotor encoder
  private TalonFXConfiguration usingRemoteEncoderConfig; // Configuration for reading a remote encoder (CANcoder)

  private VoltageOut turretVoltageControl = new VoltageOut(0.0).withEnableFOC(true);
  private PositionVoltage turretPositionControl = new PositionVoltage(0.0).withEnableFOC(true);
  private MotionMagicVoltage turretMMVoltageControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

  private boolean turretAutoAimEnabled = false;

  // Create variables for the turret Kraken motor
  private final StatusSignal<Voltage> turretSupplyVoltage; // Incoming bus voltage to motor, in volts
  private final StatusSignal<Temperature> turretTemp; // Motor temp, in degrees Celsius
  private final StatusSignal<Double> turretDutyCycle; // Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Current> turretStatorCurrent; // Motor stator current, in amps (positive = forward,
                                                           // negative = reverse)
  private final StatusSignal<Angle> turretEncoderPosition; // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> turretEncoderVelocity;
  private final StatusSignal<ControlModeValue> turretControlMode;

  // Create CANcoder
  private final CANcoder canCoder1 = new CANcoder(Ports.CANTurretEncoder1);
  private final CANcoderConfigurator canCoder1Configurator = canCoder1.getConfigurator();
  private CANcoderConfiguration canCoder1Config;
  private final CANcoder canCoder2 = new CANcoder(Ports.CANTurretEncoder2);
  private final CANcoderConfigurator canCoder2Configurator = canCoder2.getConfigurator();
  private CANcoderConfiguration canCoder2Config;

  // CANcoder signals and sensors
  private final StatusSignal<MagnetHealthValue> canCoder1MagnetHealth;
  private final StatusSignal<Angle> canCoder1Position; // CANcoder position, in CANcoder rotations
  private final StatusSignal<Angle> canCoder1AbsPosition; // CANcoder position, in CANcoder rotations
  private final StatusSignal<MagnetHealthValue> canCoder2MagnetHealth;
  private final StatusSignal<Angle> canCoder2Position; // CANcoder position, in CANcoder rotations
  private final StatusSignal<Angle> canCoder2AbsPosition; // CANcoder position, in CANcoder rotations

  // Create Data Log Entries
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogTurretTemp = new DoubleLogEntry(log, "/Turret/Motor Temp");
  private final DoubleLogEntry dLogTurretPctOut = new DoubleLogEntry(log, "/Turret/Pct Out");
  private final DoubleLogEntry dLogTurretCurrent = new DoubleLogEntry(log, "/Turret/Current (amps)");
  private final DoubleLogEntry dLogBatteryVoltage = new DoubleLogEntry(log, "/Turret/Battery Voltage");
  private final DoubleLogEntry dLogCANcoder1RotRaw = new DoubleLogEntry(log, "/Turret/CANcoder1 Rot Raw");
  private final DoubleLogEntry dLogCANcoder2RotRaw = new DoubleLogEntry(log, "/Turret/CANcoder2 Rot Raw");
  private final DoubleLogEntry dLogTurretCalZero = new DoubleLogEntry(log, "/Turret/TurretCalZero");
  private final DoubleLogEntry dLogTurretTarget = new DoubleLogEntry(log, "/Turret/Target");
  private final DoubleLogEntry dLogTurretEncAngle = new DoubleLogEntry(log, "/Turret/Encoder Angle (deg)");
  private final DoubleLogEntry dLogTurretAngle = new DoubleLogEntry(log, "/Turret/Turret Angle (Deg)");
  private final DoubleLogEntry dLogTurretVelDPS = new DoubleLogEntry(log, "/Turret/Turret Velocity dps");
  private final DoubleLogEntry dLogTurretBasicEstimation = new DoubleLogEntry(log, "/Turret/Turret Basic Estimation");
  private final DoubleLogEntry dLogTurretAdvancedEstimation = new DoubleLogEntry(log, "/Turret/Turret Advanced Estimation");
  private final BooleanLogEntry dLogTurretCalibrated = new BooleanLogEntry(log, "/Turret/Turret Calibrated?");
  private final BooleanLogEntry dLogCANcoder1Connected = new BooleanLogEntry(log, "/Turret/CANcoder1 Connected?");
  private final BooleanLogEntry dLogCANcoder2Connected = new BooleanLogEntry(log, "/Turret/CANcoder2 Connected?");

  private double turretZero = 0.0;  // offset of turret encoder (either internal motor encoder or calculated from encoder1 through gear ratio) to turret zero position, in turret degrees

  private boolean turretCalibrated = false; // Default to turret being uncalibrated. Calibrate from robot preferences or
                                            // "Calibrate Turret Zero" button on dashboard.

  private double targetAngle; // Current turret target on position control on the motor (if in position mode)


  public Turret(String subsystemName) {
    logRotationKey = DataLogUtil.allocateLogRotation();
    this.subsystemName = subsystemName;

    // Get signal and sensor objects for motor
    turretSupplyVoltage = turretMotor.getSupplyVoltage();
    turretTemp = turretMotor.getDeviceTemp();
    turretDutyCycle = turretMotor.getDutyCycle();
    turretStatorCurrent = turretMotor.getStatorCurrent();
    turretEncoderPosition = turretMotor.getPosition();
    turretEncoderVelocity = turretMotor.getVelocity();
    turretControlMode = turretMotor.getControlMode();

    // Create the motor configuration for using the rotor encoder
    usingRotorEncoderConfig = new TalonFXConfiguration();
    usingRotorEncoderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    usingRotorEncoderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    usingRotorEncoderConfig.Voltage.PeakForwardVoltage = TurretConstants.compensationVoltage;
    usingRotorEncoderConfig.Voltage.PeakReverseVoltage = -TurretConstants.compensationVoltage;
    usingRotorEncoderConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1; // Time from 0 to full power, in seconds
    usingRotorEncoderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0; // Time from 0 to full power, in seconds

    // Turn off soft limits until encoder is calibrated
    usingRotorEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    usingRotorEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // If the current is above the supply current limit for the threshold time, the
    // current is
    // limited to the lower limit in order to prevent the breakers from tripping
    usingRotorEncoderConfig.CurrentLimits.SupplyCurrentLimit = 60.0; // Upper limit for the current, in amps
    usingRotorEncoderConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0; // Lower limit for the current, in amps
    usingRotorEncoderConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2; // Threshold time, in seconds
    usingRotorEncoderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Configure PID for MotionMagicVoltage control
    usingRotorEncoderConfig.Slot0.kP = TurretConstants.kP; // kP = (desired-output-volts) / (error-in-encoder-rotations)
    usingRotorEncoderConfig.Slot0.kI = TurretConstants.kI;
    usingRotorEncoderConfig.Slot0.kD = TurretConstants.kD;
    usingRotorEncoderConfig.Slot0.kS = TurretConstants.kS;
    usingRotorEncoderConfig.Slot0.kV = TurretConstants.kV;
    usingRotorEncoderConfig.Slot0.kA = TurretConstants.kA;

    // Configure Magic Motion settings
    usingRotorEncoderConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MMCruiseVelocity;
    usingRotorEncoderConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.MMAcceleration;
    usingRotorEncoderConfig.MotionMagic.MotionMagicJerk = TurretConstants.MMJerk;

    // Configure encoder to user for feedback
    usingRotorEncoderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    usingRotorEncoderConfig.Feedback.RotorToSensorRatio = 1.0;
    usingRotorEncoderConfig.Feedback.SensorToMechanismRatio = TurretConstants.turretToMotorGearRatio;
    usingRotorEncoderConfig.ClosedLoopGeneral.ContinuousWrap = false;


    // Create configuration for using the CANcoder reading (encoder 1)
    usingRemoteEncoderConfig = new TalonFXConfiguration();
    usingRemoteEncoderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    usingRemoteEncoderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    usingRemoteEncoderConfig.Voltage.PeakForwardVoltage = TurretConstants.compensationVoltage;
    usingRemoteEncoderConfig.Voltage.PeakReverseVoltage = -TurretConstants.compensationVoltage;
    usingRemoteEncoderConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1; // Time from 0 to full power, in seconds
    usingRemoteEncoderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0; // Time from 0 to full power, in seconds

    // Turn off soft limits until encoder is calibrated
    usingRemoteEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    usingRemoteEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // If the current is above the supply current limit for the threshold time, the
    // current is
    // limited to the lower limit in order to prevent the breakers from tripping
    usingRemoteEncoderConfig.CurrentLimits.SupplyCurrentLimit = 60.0; // Upper limit for the current, in amps
    usingRemoteEncoderConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0; // Lower limit for the current, in amps
    usingRemoteEncoderConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2; // Threshold time, in seconds
    usingRemoteEncoderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Configure PID for MotionMagicVoltage control
    usingRemoteEncoderConfig.Slot0.kP = TurretConstants.kP; // kP = (desired-output-volts) / (error-in-encoder-rotations)
    usingRemoteEncoderConfig.Slot0.kI = TurretConstants.kI;
    usingRemoteEncoderConfig.Slot0.kD = TurretConstants.kD;
    usingRemoteEncoderConfig.Slot0.kS = TurretConstants.kS;
    usingRemoteEncoderConfig.Slot0.kV = TurretConstants.kV;
    usingRemoteEncoderConfig.Slot0.kA = TurretConstants.kA;

    // Configure Magic Motion settings
    usingRemoteEncoderConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MMCruiseVelocity;
    usingRemoteEncoderConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.MMAcceleration;
    usingRemoteEncoderConfig.MotionMagic.MotionMagicJerk = TurretConstants.MMJerk;

    // Configure encoder to user for feedback
    usingRemoteEncoderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    usingRemoteEncoderConfig.Feedback.FeedbackRemoteSensorID = Ports.CANTurretEncoder1;
    usingRemoteEncoderConfig.Feedback.RotorToSensorRatio = TurretConstants.encoder1ToTurretRatio * TurretConstants.turretToMotorGearRatio;
    usingRemoteEncoderConfig.Feedback.SensorToMechanismRatio = 1.0 / TurretConstants.encoder1ToTurretRatio;
    usingRemoteEncoderConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // NOTE: In Phoenix 6, slots are selected in the ControlRequest (ex.
    // PositionVoltage.Slot)
    turretPositionControl.Slot = 0;
    turretPositionControl.OverrideBrakeDurNeutral = true;
    turretMMVoltageControl.Slot = 0;
    turretMMVoltageControl.OverrideBrakeDurNeutral = true;

    // Get signal and sensor objects for CANcoder
    canCoder1MagnetHealth = canCoder1.getMagnetHealth();
    canCoder1Position = canCoder1.getPosition();
    canCoder1AbsPosition = canCoder1.getAbsolutePosition();
    canCoder2MagnetHealth = canCoder2.getMagnetHealth();
    canCoder2Position = canCoder2.getPosition();
    canCoder2AbsPosition = canCoder2.getAbsolutePosition();

    // Create CANcoder configuration
    canCoder1Config = new CANcoderConfiguration();
    canCoder1Config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoder1Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    canCoder1Config.MagnetSensor.MagnetOffset = 0.0;

    canCoder2Config = new CANcoderConfiguration();
    canCoder2Config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoder2Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    canCoder2Config.MagnetSensor.MagnetOffset = 0.0;

    // Apply the configurations to the CANcoder.
    // This is a blocking call and will wait up to 50ms-70ms for the config to
    // apply.
    canCoder1Configurator.apply(canCoder1Config);
    canCoder2Configurator.apply(canCoder2Config);

    // Use encoder1 for motion control if it is connected
    // since encoder1 has less backlash to the turret than the motor to the turret
    usingEncoder1 = isCANcoderConnected(1);
    if(usingEncoder1) {
      turretMotorConfigurator.apply(usingRemoteEncoderConfig);
      DataLogUtil.writeMessageEcho("Turret: Motor using Encoder1 for motion control.");
    } else {
      // Use motor built-in encoder for motion control if encoder1 is not connected
      turretMotorConfigurator.apply(usingRotorEncoderConfig);
      DataLogUtil.writeMessageEcho("Turret: Error -- Encoder 1 not found!  Motor using internal encoder for motion control.");

      // Record the missing encoder as a sticky fault
      RobotPreferences.recordStickyFaults("Turret-Encoders");
    }

    // NOTE!!! When the TalonFX encoder settings are changed above, then the next
    // call to getTurningEncoderDegrees()
    // may contain an old value, not the value based on the updated configuration
    // settings above!!!! The CANBus runs
    // asynchronously from this code, so sending the updated configuration to the
    // CANcoder/TalonFX and then receiving
    // an updated position measurement back may take longer than this code. The
    // timeouts in the configuration code
    // above should take care of this, but it does not always wait long enough.
    // So, add a wait time here:
    Wait.waitTime(250);

    // Calibrate if both encoders are connected
    if (areBothCANcodersConnected()) {
      calibrateTurret( calcTurretAngleFromEncodersAdvanced() );
      
    } else {
      turretCalibrated = false;

      // Record the missing encoders as a sticky fault
      RobotPreferences.recordStickyFaults("Turret-Encoders");
      DataLogUtil.writeMessageEcho("Turret: Turret not calibrated");
    }

    stopTurret();

    // Prime the DataLog to reduce delay when first enabling the robot
    updateTurretLog(true);
  }

  /**
   * Gets the name of the subsystem.
   * 
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }

  // ********** Turret movement methods

  /**
   * Stops the turret motor.
   */
  public void stopTurret() {
    setTurretPercentOutput(0.0);
  }

  /**
   * Sets the percent output of the turret motor using voltage compensation.
   * 
   * @param percent output percent, -1.0 to 1.0 (positive = CCW, negative = CW)
   */
  public void setTurretPercentOutput(double percent) {
    if (turretCalibrated) {
      percent = MathUtil.clamp(percent, -TurretConstants.maxPercentOutput, TurretConstants.maxPercentOutput);
    } else {
      percent = MathUtil.clamp(percent, -TurretConstants.maxUncalibratedPercentOutput,
          TurretConstants.maxUncalibratedPercentOutput);
    }

    turretMotor.setControl(turretVoltageControl.withOutput(percent * TurretConstants.compensationVoltage));
  }

  /**
   * Gets the percent output of the turret.
   * 
   * @return percent output percent, -1.0 to 1.0 (positive = CCW, negative = CW)
   */
  public double getTurretMotorPercentOutput() {
    turretDutyCycle.refresh();
    return turretDutyCycle.getValueAsDouble();
  }

  /**
   * Gets whether the turret is in position control or direct percent output
   * control.
   * 
   * @return true = position control, false = direct percent output control
   */
  public boolean isTurretMotorPositionControl() {
    return (turretControlMode.refresh().getValue() == ControlModeValue.PositionVoltage) ||
        (turretControlMode.refresh().getValue() == ControlModeValue.MotionMagicVoltage) ||
        (turretControlMode.refresh().getValue() == ControlModeValue.PositionVoltageFOC) ||
        (turretControlMode.refresh().getValue() == ControlModeValue.MotionMagicVoltageFOC);
  }

  /**
   * Sets the angle of the turret if the turret is calibrated.
   * 
   * @param angle target angle, per TurretContstants.TurretAngle
   */
  public void setTurretAngle(TurretAngle angle) {
    setTurretAngle(angle.value);
  }

  /**
   * Sets the angle of the turret if the turret is calibrated.
   * 
   * @param angle target angle, in degrees (0 = facing straight back out of robot,
   *              positive = counter clockwise, negative = clockwise)
   */
  public void setTurretAngle(double angle) {
    if (turretCalibrated) {
      // Normalize angle
      angle = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(angle)));


      // Phoenix6 PositionVoltage control: Position is in rotor rotations
      turretMotor.setControl(turretMMVoltageControl.withPosition(turretDegreesToEncoderRotations(angle)));

      DataLogUtil.writeMessage(subsystemName, ": setTurretAngle, Desired Angle =", angle, ", Set Angle =", angle);
      SmartDashboard.putNumber("Turret set raw rot", turretDegreesToEncoderRotations(angle));

      targetAngle = angle;
    }
  }

  /**
   * Gets the angle that the turret is trying to move to. If the turret is not
   * calibrated, then this value is -999, since we really do not know where the turret is at.
   * If the turret is in manual control mode, then this value is the actual turret position.
   * 
   * @return desired turret angle, in degrees
   */
  public double getCurrentTurretTarget() {
    double currentTarget;

    if (turretCalibrated) {
      if (isTurretMotorPositionControl()) {
        currentTarget = targetAngle;
      } else {
        // If we are not in position control mode, then we are not moving towards a target 
        // (and the target angle may be undefined). So, get the actual turret angle instead.
        currentTarget = getTurretAngle();
      }
      return currentTarget;
    } else {
      // Turret is not calibrated. Assume we are at -999 degrees, since we do not know where the turret is at.
      return -999.0;
    }
  }

  // ****** CANcoder methods

  /**
   * Gets whether the specified CANcoder is connected to the robot and reading its magnet
   * 
   * @return true = CANcoder is connected, false = CANcoder is not connected or
   *         invalid number entered
   */
  public boolean isCANcoderConnected(int CANcoder) {
    if (CANcoder == 1) {
      canCoder1MagnetHealth.refresh();
      MagnetHealthValue magnetHealth = canCoder1MagnetHealth.getValue();
      return canCoder1.isConnected() &&
          (magnetHealth == MagnetHealthValue.Magnet_Green ||
              magnetHealth == MagnetHealthValue.Magnet_Orange);
    } else if (CANcoder == 2) {
      canCoder2MagnetHealth.refresh();
      MagnetHealthValue magnetHealth = canCoder2MagnetHealth.getValue();
      return canCoder2.isConnected() &&
          (magnetHealth == MagnetHealthValue.Magnet_Green ||
              magnetHealth == MagnetHealthValue.Magnet_Orange);
    } else
      return false;
  }

  /**
   * Gets whether both CANcoders are connected and reading their magnets
   * 
   * @return true = both CANcoders connected & reading magnets, false = one or
   *         both CANcoders not connected / reading magnets
   */
  public boolean areBothCANcodersConnected() {
    return isCANcoderConnected(1) && isCANcoderConnected(2);
  }

  /**
   * Gets the raw number of rotations from the specified encoder.
   * This value will increase/decrease continuously as the encoder makes multiple rotations.
   * Calibrated so that when <b>the fractional part</b> of <i>both</i> encoders are 0, then the turret is at absolute 0
   * (but the integer part of the encoders are random depending on both the position of the turret at robot hard boot
   * time and the position of the turret at robot code initialize time).
   * 
   * @return raw encoder reading, in encoder rotations. + is counter-clockwise, - is clockwise (when viewed from above facing turret zero)
   */
  public double getCANcoderRotationsRaw(int CANcoder) {
    if (CANcoder == 1) {
      canCoder1Position.refresh(); // Verified that this is not a blocking call
      return canCoder1Position.getValueAsDouble() - TurretConstants.offsetAngleCANcoder1;
    } else if (CANcoder == 2) {
      canCoder2Position.refresh(); // Verified that this is not a blocking call
      return canCoder2Position.getValueAsDouble() - TurretConstants.offsetAngleCANcoder2;
    } else
      return 0.0;
  }

  /**
   * Gets the raw absolute position of the specified encoder. Output will be [0, 1).
   * Calibrated so that when *both* encoders are 0, then the turret is at absolute 0.
   * 
   * @return raw absolute encoder reading, in encoder rotations. + is counter-clockwise, - is clockwise (when viewed from above facing turret zero)
   */
  public double getCANcoderAbsRotationsRaw(int CANcoder) {
    if (CANcoder == 1) {
      canCoder1AbsPosition.refresh(); // Verified that this is not a blocking call
      double angle = canCoder1AbsPosition.getValueAsDouble() - TurretConstants.offsetAngleCANcoder1;
      return (angle<0) ? angle+1 : angle;
    } else if (CANcoder == 2) {
      canCoder2AbsPosition.refresh(); // Verified that this is not a blocking call
      double angle = canCoder2AbsPosition.getValueAsDouble() - TurretConstants.offsetAngleCANcoder2;
      return (angle<0) ? angle+1 : angle;
    } else
      return 0.0;
  }

  // ****** Kraken encoder methods (may be reading internal encoder or may be reading remote CANcoder)

  /**
   * Gets the raw number of turret rotations from the encoder configured on the motor, which could be the motor encoder or encoder1.
   * 
   * @return raw turret encoder reading, in turret rotations. + is counter-clockwise, - is clockwise (when viewed from above facing turret zero)
   */
  public double getTurretEncoderRotationsRaw() {
    turretEncoderPosition.refresh(); // Verified that this is not a blocking call
    return turretEncoderPosition.getValueAsDouble();
  }

  /**
   * Gets the current turret angle from the encoder configured on the motor,
   * which could be the motor encoder or encoder1, assuming the turret is calibrated.
   * NOTE: This is for internal subsystem use only. Use getTurretAngle() when calling from outside.
   * 
   * @return calibrated turret position, in degrees. 0 = facing back of robot, + is counter-clockwise, - is clockwise (when viewed from above facing turret zero)
   */
  private double getTurretEncoderDegrees() {
    return getTurretEncoderRotationsRaw() * TurretConstants.turretDegreesPerRotation - turretZero;
  }

  /**
   * Returns the angle that the turret is currently positioned at. If the turret
   * is not calibrated, then this value is -999, since we really do not know where the turret is at.
   * 
   * @return current turret angle, in degrees (0 = facing back of robot,
   *         + is counter-clockwise, negative is clockwise (when viewed from above facing turret zero))
   */
  public double getTurretAngle() {
    if (turretCalibrated)
      return getTurretEncoderDegrees();
    else
      return -999.0;
  }

  /**
   * Converts the turret angle from degrees to raw rotations, assuming that the turret is calibrated.
   * 
   * @param degrees turret angle, in degrees
   * @return turret angle, in rotations
   */
  private double turretDegreesToEncoderRotations(double degrees) {
    return (degrees + turretZero) / TurretConstants.turretDegreesPerRotation;
  }

  // ********** Internal Kraken calibration methods

  /**
   * Gets whether the turret is calibrated.
   * 
   * @return true = turret is calibrated and working, false = turret is not calibrated
   */
  public boolean isTurretCalibrated() {
    return turretCalibrated;
  }

  /**
   * Stops the turret, sets turretCalibrated to false, and disables software limits.
   */
  public void setTurretUncalibrated() {
    stopTurret();
    turretCalibrated = false;
    turretAutoAimEnabled = false;

    // If the turret goes uncalibrated, we still want to be able to manually drive it in order to recalibrate
    usingRemoteEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    usingRemoteEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    
    usingRotorEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    usingRotorEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // Apply the configurations to the motor.
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
    if (usingEncoder1) {
      turretMotorConfigurator.apply(usingRemoteEncoderConfig);
    } else {
      turretMotorConfigurator.apply(usingRotorEncoderConfig);
    }

    DataLogUtil.writeMessageEcho("Turret: Uncalibrate, Enc Raw =", getTurretEncoderRotationsRaw(), ", Angle =",
        getTurretAngle(), ", Target =", getCurrentTurretTarget());
  }

  /**
   * Calibrates the turret assuming we know its current angle and sets turretCalibrated to true.
   * 
   * @param angle current angle the turret is physically at, in degrees
   */
  public void calibrateTurret(double angle) {
    stopTurret(); // Stop the motor so it does not jump to a new value
    turretZero = getTurretEncoderRotationsRaw() * TurretConstants.turretDegreesPerRotation - angle;
    turretCalibrated = true;

    DataLogUtil.writeMessageEcho("Turret: Calibrated, Using encoder1 =", usingEncoder1,
        ", encoder1 abs =", getCANcoderAbsRotationsRaw(1), ", encoder2 abs =", getCANcoderAbsRotationsRaw(2),
        ", turret Zero =", turretZero, ", turret Raw =", getTurretEncoderRotationsRaw(), ", Angle =", getTurretAngle(),
        ", Target =", getCurrentTurretTarget());

    if (usingEncoder1) {
      // Set software limits after setting turretZero
      usingRemoteEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = turretDegreesToEncoderRotations(
          TurretAngle.UPPER_LIMIT.value);
      usingRemoteEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = turretDegreesToEncoderRotations(
          TurretAngle.LOWER_LIMIT.value);
      usingRemoteEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      usingRemoteEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

      // Apply the configurations to the motor.
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
      turretMotorConfigurator.apply(usingRemoteEncoderConfig);
    } else {
      // Set software limits after setting turretZero
      usingRotorEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = turretDegreesToEncoderRotations(
          TurretAngle.UPPER_LIMIT.value);
      usingRotorEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = turretDegreesToEncoderRotations(
          TurretAngle.LOWER_LIMIT.value);
      usingRotorEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      usingRotorEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

      // Apply the configurations to the motor.
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
      turretMotorConfigurator.apply(usingRotorEncoderConfig);
    }
    turretAutoAimEnabled = true;
  }

  /**
   * Calibrate turret using encoder 1 and 2 readings if they are both connected, otherwise do nothing.
   */
  public void calibrateTurretAutomatic() {
    if (areBothCANcodersConnected()) {
      if (!usingEncoder1) {
        usingEncoder1 = true;
        // Apply the configurations to the motor.
        // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
        turretMotorConfigurator.apply(usingRemoteEncoderConfig);

        // NOTE!!! The call above needs time to be fully applied before we read
        // the turret angle in calibrateTurret(), so add a wait time here:
        Wait.waitTime(250);
      }
      calibrateTurret(calcTurretAngleFromEncodersAdvanced());
    } else {
      DataLogUtil.writeMessage("Turret: Automatic Calibration Failed, encoder 1 connected =", 
        isCANcoderConnected(1), " encoder 2 connected =", isCANcoderConnected(2));
    }
  }

  private double cancoderRotationsToDegrees(double rot) {
    return rot * 360.0;
  }

  /**
   * Turret angle estimation for use on initial robot boot.
   * Calculate turret angle based on the raw readings of both encoders.
   * @return Turret angle, in degrees (from -180*theoreticalCalculationCoefficient to +180*theoreticalCalculationCoefficient)
   */
  public double calcTurretAngleFromEncodersBasic() {
    double enc1angle = cancoderRotationsToDegrees(getCANcoderAbsRotationsRaw(1));
    double enc2angle = cancoderRotationsToDegrees(getCANcoderAbsRotationsRaw(2));
    
    double angle = ((enc2angle - enc1angle) * TurretConstants.encoderCalculationCoefficient);
    // Fix angle when enc2 wraps before enc1
    if (enc2angle < enc1angle) {
      angle +=  360.0 * TurretConstants.encoderCalculationCoefficient;
    }

    // Convert angle from (0, maxAngle) to (-maxAngle/2, +maxAngle/2)
    if (angle > 180.0 * TurretConstants.encoderCalculationCoefficient) {
      angle -= 360.0 * TurretConstants.encoderCalculationCoefficient;
    }

    return angle;
  }

  /**
   * Turret angle estimation for use on initial robot boot.
   * Calculate turret angle based on the raw reading of encoder 1, with correction for encoder 1 wraps by using both encoders.
   * @return Turret angle, in degrees (from -180 * theoreticalCalculationCoefficient to +180 * theoreticalCalculationCoefficient)
   */
  public double calcTurretAngleFromEncodersAdvanced() {
    double uncorrectedEncoderReading = cancoderRotationsToDegrees(getCANcoderAbsRotationsRaw(1)) * TurretConstants.encoder1ToTurretRatio;
    
    return uncorrectedEncoderReading + 360.0 * TurretConstants.encoder1ToTurretRatio * 
          (double) Math.round((calcTurretAngleFromEncodersBasic() - uncorrectedEncoderReading) /
                              360.0 / TurretConstants.encoder1ToTurretRatio);
  }

  /**
   * Gets the absolute rotation of the turret on the field.
   * @param robotRotation2d Rotation2d object for robot's current rotation.
   * @return Turret absolute field angle, as a Rotation2d object.
   */
  private Rotation2d getTurretAbsoluteAngle(Rotation2d robotRotation2d) {
    double ret = robotRotation2d.getDegrees() + calcTurretAngleFromEncodersAdvanced() + 180.0;

    // make it wrap: (-180.0, 180.0)
    ret = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(ret)));

    return new Rotation2d(Math.toRadians(ret));
  }

  /**
   * Gets the absolute position of the turret on the field.
   * @param robotPose2d Pose2d object for robot's current position.
   * @return Turret absolute field position, as a Pose2d object.
   */
  private Pose2d getTurretAbsolutePose(Pose2d robotPose2d) {
    double robotRotRadians = robotPose2d.getRotation().getRadians();
    double xOffsetAbsolute;
    double yOffsetAbsolute;

    if (robotRotRadians >= 0) {   // rotated CCW facing red wall or CW facing blue wall
      if (robotRotRadians <= Math.PI / 2.0) {   // facing red wall
        xOffsetAbsolute = TurretConstants.turretOffset * Math.cos(robotRotRadians);  // -x, so do nothing here
        yOffsetAbsolute = TurretConstants.turretOffset * Math.sin(robotRotRadians);  // -y, so do nothing here
      } else {   // facing blue wall
        xOffsetAbsolute = -1.0 * TurretConstants.turretOffset * Math.sin(Math.PI / 2.0 - robotRotRadians);  // +x, so make this negative
        yOffsetAbsolute = TurretConstants.turretOffset * Math.cos(Math.PI / 2.0 - robotRotRadians);         // -y, so do nothing here
      }
    } else {   // rotated CW facing red wall or CCW facing blue wall
      robotRotRadians = -1.0 * robotRotRadians;   // make the math easier by making angle positive
      if (robotRotRadians >= Math.PI / -2.0) {   // facing red wall
        xOffsetAbsolute = TurretConstants.turretOffset * Math.cos(robotRotRadians);         // -x, so do nothing here
        yOffsetAbsolute = -1.0 * TurretConstants.turretOffset * Math.sin(robotRotRadians);  // +y, so make this negative
      } else {   // facing blue wall
        xOffsetAbsolute = -1.0 * TurretConstants.turretOffset * Math.sin(Math.PI / 2.0 - robotRotRadians);  // +x, so make this negative
        yOffsetAbsolute = -1.0 * TurretConstants.turretOffset * Math.cos(Math.PI / 2.0 - robotRotRadians);  // +y, so make this negative
      }
    }

    // some are made negative above so we don't have to have a bunch of separate return statements!
    return new Pose2d(robotPose2d.getX() - xOffsetAbsolute, robotPose2d.getY() - yOffsetAbsolute, getTurretAbsoluteAngle(robotPose2d.getRotation()));
  }

  /**
   * Get turret target angle for aiming to the hub based on current alliance, robot position/rotation, and turret rotation.
   * @param alliance Current Alliance
   * @param robotPose2d Pose2d object for robot's current position
   * @return goal auto-targeting angle based on current turret position, in degrees
   */
  private double getAutoTargetAngleAbsolute(Alliance alliance, Pose2d robotPose2d) {
    double ret;

    if (alliance == Alliance.Blue) {
      if (robotPose2d.getX() < FieldConstants.hubPositionBlue.getX()) {
        ret = MathUtil.angleModulus(Math.atan((getTurretAbsolutePose(robotPose2d).getY() - FieldConstants.hubPositionBlue.getY()) / (getTurretAbsolutePose(robotPose2d).getX() - FieldConstants.hubPositionBlue.getX())));
      } else {
        if (robotPose2d.getY() > FieldConstants.width / 2.0) {
          ret = Math.PI + MathUtil.angleModulus(Math.atan((getTurretAbsolutePose(robotPose2d).getY() - FieldConstants.leftPassBlue.getY()) / (getTurretAbsolutePose(robotPose2d).getX() - FieldConstants.leftPassBlue.getX())));
        } else {
          ret = Math.PI + MathUtil.angleModulus(Math.atan((getTurretAbsolutePose(robotPose2d).getY() - FieldConstants.rightPassBlue.getY()) / (getTurretAbsolutePose(robotPose2d).getX() - FieldConstants.rightPassBlue.getX())));
        }
      }
    } else {
      if (robotPose2d.getX() > FieldConstants.hubPositionRed.getX()) {
        ret = Math.PI + MathUtil.angleModulus(Math.atan((getTurretAbsolutePose(robotPose2d).getY() - FieldConstants.hubPositionRed.getY()) / (getTurretAbsolutePose(robotPose2d).getX() - FieldConstants.hubPositionRed.getX())));
      } else {
        if (robotPose2d.getY() > FieldConstants.width / 2.0) {
          ret = -MathUtil.angleModulus(Math.atan((getTurretAbsolutePose(robotPose2d).getY() - FieldConstants.rightPassRed.getY()) / (getTurretAbsolutePose(robotPose2d).getX() - FieldConstants.rightPassRed.getX())));
        } else {
          ret = -MathUtil.angleModulus(Math.atan((getTurretAbsolutePose(robotPose2d).getY() - FieldConstants.leftPassRed.getY()) / (getTurretAbsolutePose(robotPose2d).getX() - FieldConstants.leftPassRed.getX())));
        }
      }
    }

    return Math.toDegrees(ret);
  }

  /**
   * Get target angle for turret to turn to relative to turret's absolute zero.
   * @param alliance Current Alliance
   * @param robotPose2d Current Pose2d of robot on field
   * @return target angle for turret to turn to
   */
  public double getAutoTargetAngleTurretRelative(Alliance alliance, Pose2d robotPose2d) {
    return Math.toDegrees(MathUtil.angleModulus(Math.toRadians(getAutoTargetAngleAbsolute(alliance, robotPose2d) - robotPose2d.getRotation().getDegrees() - 180.0)));
  }

  /**
   * Toggle whether or not turret should be auto aiming
   */
  public void toggleAutoAim() {
    turretAutoAimEnabled = !turretAutoAimEnabled;
  }

  /** 
   * Return current auto aim state
   * @return true = auto aim, false = manual control
   */
  public boolean getAutoAimEnabled() {
    return turretAutoAimEnabled;
  }

  // ****** Information methods

  /**
   * Turns file logging on every scheduler cycle (~20 ms) or every 10 cycles (~0.2 sec).
   * 
   * @param enabled true = log every cycle, false = log every 10 cycles
   */
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Writes information about the turret to the file log.
   * 
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateTurretLog(boolean logWhenDisabled) {
    if (logWhenDisabled || !DriverStation.isDisabled()) {
      long timeNow = RobotController.getFPGATime();

      dLogTurretTemp.append(turretTemp.refresh().getValueAsDouble(), timeNow);
      dLogTurretPctOut.append(getTurretMotorPercentOutput(), timeNow);
      dLogTurretCurrent.append(turretStatorCurrent.refresh().getValueAsDouble(), timeNow);
      dLogBatteryVoltage.append(turretSupplyVoltage.refresh().getValueAsDouble(), timeNow);
      dLogCANcoder1RotRaw.append(getCANcoderRotationsRaw(1), timeNow);
      dLogCANcoder2RotRaw.append(getCANcoderRotationsRaw(2), timeNow);
      dLogTurretCalZero.append(turretZero, timeNow);
      dLogTurretTarget.append(getCurrentTurretTarget(), timeNow);
      dLogTurretEncAngle.append(getTurretEncoderDegrees(), timeNow);
      dLogTurretAngle.append(getTurretAngle(), timeNow);
      dLogTurretVelDPS.append(turretEncoderVelocity.refresh().getValueAsDouble() * TurretConstants.turretDegreesPerRotation, timeNow);
      dLogTurretBasicEstimation.append(calcTurretAngleFromEncodersBasic(), timeNow);
      dLogTurretAdvancedEstimation.append(calcTurretAngleFromEncodersAdvanced(), timeNow);
      dLogTurretCalibrated.append(isTurretCalibrated(), timeNow);
      dLogCANcoder1Connected.append(isCANcoderConnected(1), timeNow);
      dLogCANcoder2Connected.append(isCANcoderConnected(2), timeNow);
    }
  }

  @Override
  public void periodic() {

    if (DataLogUtil.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean(TURRET_IS_CANCODER_CONNECTED.value, areBothCANcodersConnected());
      SmartDashboard.putBoolean(TURRET_IS_CALIBRATED.value, turretCalibrated);
      SmartDashboard.putBoolean(TURRET_IS_USING_CANCODER.value, usingEncoder1);
      SmartDashboard.putNumber(TURRET_CANCODER1_RAW.value, getCANcoderRotationsRaw(1));
      SmartDashboard.putNumber(TURRET_CANCODER2_RAW.value, getCANcoderRotationsRaw(2));
      SmartDashboard.putNumber(TURRET_CANCODER1_ABS.value, getCANcoderAbsRotationsRaw(1));
      SmartDashboard.putNumber(TURRET_CANCODER2_ABS.value, getCANcoderAbsRotationsRaw(2));
      SmartDashboard.putNumber(TURRET_ENCODER_RAW.value, getTurretEncoderRotationsRaw());
      SmartDashboard.putNumber(TURRET_ANGLE.value, getTurretEncoderDegrees());
      SmartDashboard.putNumber(TURRET_TARGET_ANGLE.value, getCurrentTurretTarget());

      SmartDashboard.putNumber(TURRET_ADVANCED_ESTIMATION.value, calcTurretAngleFromEncodersAdvanced());
    }

    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      updateTurretLog(false);
    }

    // If the CANcoder stops reading, apply RotorEncoder configuration and stop
    // using the CANcoder
    // This condition should only occur one time (if at all)
    if (usingEncoder1 && !isCANcoderConnected(1)) {
      usingEncoder1 = false;
      setTurretUncalibrated(); // will make a blocking call (50-70ms)

      RobotPreferences.recordStickyFaults("Turret-CANcoder");

      DataLogUtil.writeMessageEcho("Turret: CANcoder Disconnection, Enc Zero =", turretZero,
          ", CANcoder1 Angle =", getCANcoderRotationsRaw(1),
          ", CANcoder2 Angle =", getCANcoderRotationsRaw(2),
          ", CANcoder1 Connected =", isCANcoderConnected(1),
          ", CANcoder2 Connected =", isCANcoderConnected(2),
          ", Enc Raw =", getTurretEncoderRotationsRaw(),
          ", Angle =", getTurretEncoderDegrees(),
          ", Target =", getCurrentTurretTarget());
    }

    // If the driver station is disabled, then turn off any position control for the
    // turret motor
    if (DriverStation.isDisabled()) {
      stopTurret();
    }
  }
}
