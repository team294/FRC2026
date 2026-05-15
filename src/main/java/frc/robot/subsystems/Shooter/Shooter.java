// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.DataLogUtil.RotationWhenDisabled;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class Shooter extends SubsystemBase implements Loggable {
    private final int logRotationKey;
    private boolean fastLogging = true;
    private final String subsystemName;

    // AdvantageKit IO layer
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // Shooter state
    private boolean shooterVelocityControlOn = false;
    private double shooterSetpointRPM = 0.0;

    // Hood state
    // PID lives here (not in IO) because it is not hardware-specific.
    private final PIDController hood1PID = new PIDController(
        ShooterConstants.hood1kP, ShooterConstants.hood1kI, ShooterConstants.hood1kD);
    private final PIDController hood2PID = new PIDController(
        ShooterConstants.hood2kP, ShooterConstants.hood2kI, ShooterConstants.hood2kD);

    private double hoodAngleSetpoint = 0.0;
    private double hood1VoltageSetpoint = 0.0;
    private double hood2VoltageSetpoint = 0.0;
    private boolean hood1PositionControlOn = false;
    private boolean hood2PositionControlOn = false;

    // Sticky fault tracking
    private boolean recordedShooter1StickyFault = false;
    private boolean recordedShooter2StickyFault = false;
    private boolean recordedHood1StickyFault = false;
    private boolean recordedHood2StickyFault = false;

    /**
     * Constructs the Shooter subsystem.
     *
     * @param io ShooterIO implementation (real or sim)
     * @param subsystemName Name used for logging and dashboard
     */
    public Shooter(ShooterIO io, String subsystemName) {
        this.io = io;
        this.subsystemName = subsystemName;

        logRotationKey = DataLogUtil.allocateLogRotation();

        hood1PID.reset();
        hood2PID.reset();
        hood1PID.setTolerance(ShooterConstants.hoodPIDErrorTolerance);
        hood2PID.setTolerance(ShooterConstants.hoodPIDErrorTolerance);

        stopShooterMotor();
        setHoodVoltageBothMotors(0.0);

        DataLogUtil.writeMessage(subsystemName, ": Constructed");
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(subsystemName, inputs);

        // Enforce hood voltage cutoffs at the limits of the linear actuators
        enforceHoodCutoffs();

        if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kSlow)) {
            if (DriverStation.isEnabled()) {
                Logger.recordOutput(subsystemName + "/ShooterVelocitySetpointRPM", shooterSetpointRPM);
                Logger.recordOutput(subsystemName + "/ShooterVelocityControlOn", shooterVelocityControlOn);
                Logger.recordOutput(subsystemName + "/HoodAngleSetpointDeg", hoodAngleSetpoint);
                Logger.recordOutput(subsystemName + "/HoodAngleDeg", getHoodAngle());
                Logger.recordOutput(subsystemName + "/Hood1AngleDeg", getHoodAngle(1));
                Logger.recordOutput(subsystemName + "/Hood2AngleDeg", getHoodAngle(2));
                Logger.recordOutput(subsystemName + "/Hood1VoltageSetpoint", hood1VoltageSetpoint);
                Logger.recordOutput(subsystemName + "/Hood2VoltageSetpoint", hood2VoltageSetpoint);
                Logger.recordOutput(subsystemName + "/HoodPositionControlOn", hood2PositionControlOn);
            }
        }

        if (DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kNormal)) {
            checkForFaultConditions();

            SmartDashboard.putNumber(SHOOTER_LEFT_TEMP.value, inputs.shooter1TempCelsius);
            SmartDashboard.putNumber(SHOOTER_RIGHT_TEMP.value, inputs.shooter2TempCelsius);
            SmartDashboard.putNumber(SHOOTER_RPM.value, getShooterVelocity());
            SmartDashboard.putNumber(SHOOTER_TARGET_RPM.value, shooterSetpointRPM);

            SmartDashboard.putNumber(HOOD_LEFT_VOLTAGE.value, inputs.hood1AppliedVolts);
            SmartDashboard.putNumber(HOOD_RIGHT_VOLTAGE.value, inputs.hood2AppliedVolts);
            SmartDashboard.putNumber(HOOD_LEFT_ANALOG_VOLTAGE.value, inputs.hood1AnalogVoltage);
            SmartDashboard.putNumber(HOOD_RIGHT_ANALOG_VOLTAGE.value, inputs.hood2AnalogVoltage);
            SmartDashboard.putNumber(HOOD_LEFT_LENGTH.value, getHoodExtensionLength(1));
            SmartDashboard.putNumber(HOOD_RIGHT_LENGTH.value, getHoodExtensionLength(2));
            SmartDashboard.putNumber(HOOD_LEFT_ANGLE.value, getHoodAngle(1));
            SmartDashboard.putNumber(HOOD_RIGHT_ANGLE.value, getHoodAngle(2));
            SmartDashboard.putNumber(HOOD_ANGLE.value, getHoodAngle());
            SmartDashboard.putNumber(HOOD_ANGLE_CURRENT_SETPOINT.value, hoodAngleSetpoint);
        }
    }


    // Shooter methods

    /**
     * Sets the percent output of the shooter motors using voltage compensation.
     * @param percent output percent, -1.0 to 1.0 (positive = shoot direction, negative = reverse)
     */
    public void setShooterPercentOutput(double percent) {
        shooterVelocityControlOn = false;
        shooterSetpointRPM = 0.0;
        io.setShooterVoltage(percent * ShooterConstants.compensationVoltage);
    }

    /**
     * Sets the velocity of the shooter wheel.
     * @param velocityRPM target velocity in RPM (positive = shoot, negative = reverse)
     */
    public void setShooterVelocity(double velocityRPM) {
        shooterVelocityControlOn = true;
        shooterSetpointRPM = velocityRPM;
        double motorRPS = (velocityRPM / 60.0) / ShooterConstants.gearRatio;
        io.setShooterVelocity(motorRPS);
    }

    /**
     * Stops the shooter motors.
     */
    public void stopShooterMotor() {
        setShooterPercentOutput(0.0);
    }

    /**
     * Gets the velocity of one shooter motor at the wheel.
     * @param motorNumber which motor: 1 or 2
     * @return wheel velocity, in RPM
     */
    public double getShooterVelocity(int motorNumber) {
        double rps = motorNumber == 1 ? inputs.shooter1VelocityRPS : inputs.shooter2VelocityRPS;
        return rps * ShooterConstants.gearRatio * 60.0;
    }

    /**
     * Gets the average wheel velocity of both shooter motors.
     * @return wheel velocity, in RPM
     */
    public double getShooterVelocity() {
        return (getShooterVelocity(1) + getShooterVelocity(2)) / 2.0;
    }

    /**
     * Gets the encoder position of one shooter motor at the wheel.
     * @param motorNumber which motor: 1 or 2
     * @return position in wheel rotations
     */
    public double getShooterEncoderPosition(int motorNumber) {
        double rotations = motorNumber == 1 ? inputs.shooter1PositionRotations : inputs.shooter2PositionRotations;
        return rotations * ShooterConstants.gearRatio;
    }

    /**
     * Gets the stator current of one shooter motor.
     * @param motorNumber which motor: 1 or 2
     * @return stator current, in amps
     */
    public double getShooterAmps(int motorNumber) {
        return motorNumber == 1 ? inputs.shooter1CurrentAmps : inputs.shooter2CurrentAmps;
    }

    /**
     * Gets the applied voltage of one shooter motor.
     * @param motorNumber which motor: 1 or 2
     * @return applied voltage, in volts
     */
    public double getShooterVoltage(int motorNumber) {
        return motorNumber == 1 ? inputs.shooter1AppliedVolts : inputs.shooter2AppliedVolts;
    }

    /**
     * Gets the temperature of one shooter motor.
     * @param motorNumber which motor: 1 or 2
     * @return temperature, in degrees Celsius
     */
    public double getShooterTemp(int motorNumber) {
        return motorNumber == 1 ? inputs.shooter1TempCelsius : inputs.shooter2TempCelsius;
    }

    /**
     * Returns whether velocity control is currently active.
     * @return true = velocity control on, false = percent output or stopped
     */
    public boolean isShooterVelocityControlOn() {
        return shooterVelocityControlOn;
    }

    /**
     * Returns the current shooter velocity setpoint.
     * @return setpoint in RPM
     */
    public double getShooterSetpointRPM() {
        return shooterSetpointRPM;
    }

    /**
     * Gets the velocity PID error for the shooter wheel.
     * @return error in RPM (positive = measured too fast, negative = too slow)
     */
    public double getShooterVelocityPIDError() {
        return getShooterVelocity() - shooterSetpointRPM;
    }

    /**
     * Returns whether the shooter is within tolerance of its goal velocity.
     * @return true = at goal, false = not yet at goal velocity
     */
    public boolean isShooterAtGoalVelocity() {
        return shooterSetpointRPM != 0.0
            && Math.abs(getShooterVelocityPIDError()) < ShooterConstants.shooterVelocityErrorTolerance;
    }


    // Hood methods

    /**
     * Reads the hood analog voltage for one motor.
     * @param motorNumber which motor: 1 or 2
     * @return analog voltage, in volts (0 – 5)
     */
    public double getHoodAnalogVoltage(int motorNumber) {
        return motorNumber == 1 ? inputs.hood1AnalogVoltage : inputs.hood2AnalogVoltage;
    }

    /**
     * Reads the average hood analog voltage of both motors.
     * @return analog voltage, in volts
     */
    public double getHoodAnalogVoltage() {
        return (getHoodAnalogVoltage(1) + getHoodAnalogVoltage(2)) / 2.0;
    }

    /**
     * Sets both hood linear actuators to the same voltage.
     * @param voltage in volts [-12, 12]; positive = up, negative = down
     */
    public void setHoodVoltageBothMotors(double voltage) {
        hood1VoltageSetpoint = voltage;
        hood2VoltageSetpoint = voltage;
        hood1PositionControlOn = false;
        hood2PositionControlOn = false;
        io.setHood1Voltage(voltage * ShooterConstants.hoodMotorRatio2to1);
        io.setHood2Voltage(voltage);
    }

    /**
     * Sets the voltage for one hood linear actuator.
     * @param voltage in volts [-12, 12]; positive = up, negative = down
     * @param motorNumber which motor: 1 or 2
     */
    public void setHoodVoltageOneMotor(double voltage, int motorNumber) {
        if (motorNumber == 1) {
            hood1VoltageSetpoint = voltage;
            hood1PositionControlOn = false;
            io.setHood1Voltage(voltage);
        } else {
            hood2VoltageSetpoint = voltage;
            hood2PositionControlOn = false;
            io.setHood2Voltage(voltage);
        }
    }

    /**
     * Moves both hood motors toward an angle target using software PID.
     * <p><b>Note:</b> must be called every execute() cycle to sustain PID control.
     * @param angle target angle, in degrees
     */
    public void setHoodAnglePID(double angle) {
        double safeAngle = MathUtil.clamp(angle, ShooterConstants.hoodMinimumAngle, ShooterConstants.hoodMaximumAngle);
        hoodAngleSetpoint = safeAngle;
        hood1PositionControlOn = true;
        hood2PositionControlOn = true;

        if (Math.abs(getHoodAngle() - hoodAngleSetpoint) < ShooterConstants.hoodAngleErrorTolerance / 2.0) {
            // Very close to target — hold still to avoid buzzing
            io.setHood1Voltage(0.0);
            io.setHood2Voltage(0.0);
        } else {
            // HoodFix: left (#1) analog sensor is faulty; drive both motors from motor 2's reading,
            // applying a calibration ratio to motor 1 to compensate for mechanical differences.
            double hoodPct = hood2PID.calculate(getHoodAngle(2), hoodAngleSetpoint);
            io.setHood1Voltage(hoodPct * ShooterConstants.hoodMotorRatio2to1 * ShooterConstants.compensationVoltage);
            io.setHood2Voltage(hoodPct * ShooterConstants.compensationVoltage);
        }
    }

    /**
     * Gets the estimated linear actuator extension length for one motor, including the base of the actuator.
     * @param motorNumber which motor: 1 or 2
     * @return estimated extension length, in inches
     */
    public double getHoodExtensionLength(int motorNumber) {
        double slope = motorNumber == 1
            ? ShooterConstants.hood1VoltageToExtensionLengthSlope
            : ShooterConstants.hood2VoltageToExtensionLengthSlope;
        return slope * getHoodAnalogVoltage(motorNumber) + ShooterConstants.hoodActuatorMinExtensionLength;
    }

    /**
     * Gets the average estimated extension length across both hood motors.
     * @return average extension length, in inches
     */
    public double getHoodExtensionLength() {
        return (getHoodExtensionLength(1) + getHoodExtensionLength(2)) / 2.0;
    }

    /**
     * Estimates the current hood angle from one motor's analog voltage reading.
     * @param motorNumber which motor: 1 or 2
     * @return estimated hood angle, in degrees
     */
    public double getHoodAngle(int motorNumber) {
        double extension = getHoodExtensionLength(motorNumber);
        double cosArg = ((ShooterConstants.radius1 * ShooterConstants.radius1)
            + (ShooterConstants.radius2 * ShooterConstants.radius2)
            - (extension * extension))
            / (2.0 * ShooterConstants.radius1 * ShooterConstants.radius2);
        return Math.toDegrees(Math.acos(cosArg)) - (54.8122837191 - ShooterConstants.hoodMinimumAngle);
    }

    /**
     * Gets the estimated current hood angle.
     * Uses motor 2's reading only, due to a known issue with motor 1's analog sensor.
     * @return estimated hood angle, in degrees
     */
    public double getHoodAngle() {
        // HoodFix: motor 1 analog sensor is flaky — use motor 2 exclusively.
        return getHoodAngle(2);
    }

    /**
     * Converts a desired hood angle to the corresponding linear actuator extension length.
     * @param angle desired hood angle, in degrees
     * @return linear actuator extension, in inches
     */
    public double getHoodAngleToExtension(double angle) {
        double cosArg = Math.cos(Math.toRadians(angle + 54.8122837191 - ShooterConstants.hoodMinimumAngle))
            * (2.0 * ShooterConstants.radius1 * ShooterConstants.radius2);
        return Math.sqrt(
            (ShooterConstants.radius2 * ShooterConstants.radius2)
                + (ShooterConstants.radius1 * ShooterConstants.radius1)
                - cosArg)
            + ShooterConstants.hoodActuatorMinExtensionLength;
    }

    /**
     * Returns the current hood angle setpoint.
     * @return setpoint in degrees
     */
    public double getHoodAngleSetpoint() {
        return hoodAngleSetpoint;
    }

    /**
     * Returns whether the hood is within tolerance of its goal angle.
     * @return true = at goal, false = not yet at goal angle
     */
    public boolean isHoodAtGoalAngle() {
        return Math.abs(getHoodAngle() - hoodAngleSetpoint) < ShooterConstants.hoodAngleErrorTolerance;
    }

    /**
     * Returns whether the hood is in a safe position for trench traversal.
     * @return true = both actuators are near minimum angle
     */
    public boolean isHoodSafeForTrench() {
        return Math.abs(getHoodAngle(1) - ShooterConstants.hoodMinimumAngle) < ShooterConstants.hoodAngleErrorTolerance
            && Math.abs(getHoodAngle(2) - ShooterConstants.hoodMinimumAngle) < ShooterConstants.hoodAngleErrorTolerance;
    }


    // Information methods

    /**
     * Gets the name of the subsystem.
     * @return the subsystem name
     */
    public String getName() {
        return subsystemName;
    }

    /**
     * Re-applies full hardware configuration. Use to recover from a brownout.
     * <p><b>NOTE:</b> includes blocking CAN calls.
     */
    public void configShooter() {
        io.configShooter();
    }

    /**
     * Enables or disables fast logging (every cycle vs. every 10 cycles).
     * @param enabled true = log every cycle, false = log every 10 cycles
     */
    @Override
    public void enableFastLogging(boolean enabled) {
        fastLogging = enabled;
    }


    // Private helpers

    /**
     * Cuts hood motor voltage when the actuator reaches its physical limits,
     * preventing motor damage. Mirrors the original periodic() limit logic.
     */
    private void enforceHoodCutoffs() {
        if (hood1PositionControlOn) {
            boolean atMin = getHoodAnalogVoltage(1) < ShooterConstants.hoodMinimumAnalogVoltageCutoff
                && hoodAngleSetpoint <= ShooterConstants.hoodMinimumAngle + 0.01;
            boolean atMax = getHoodAnalogVoltage(1) > ShooterConstants.hood1MaximumAnalogVoltageCutoff
                && hoodAngleSetpoint >= ShooterConstants.hoodMaximumAngle - 0.01;
            if (atMin || atMax) {
                setHoodVoltageOneMotor(0.0, 1);
            }
        } else {
            boolean pushingIntoMin = getHoodAnalogVoltage(1) < ShooterConstants.hoodMinimumAnalogVoltageCutoff
                && hood1VoltageSetpoint < 0;
            boolean pushingIntoMax = getHoodAnalogVoltage(1) > ShooterConstants.hood1MaximumAnalogVoltageCutoff
                && hood1VoltageSetpoint > 0;
            if (pushingIntoMin || pushingIntoMax) {
                setHoodVoltageOneMotor(0.0, 1);
            }
        }

        if (hood2PositionControlOn) {
            boolean atMin = getHoodAnalogVoltage(2) < ShooterConstants.hoodMinimumAnalogVoltageCutoff
                && hoodAngleSetpoint <= ShooterConstants.hoodMinimumAngle + 0.01;
            boolean atMax = getHoodAnalogVoltage(2) > ShooterConstants.hood2MaximumAnalogVoltageCutoff
                && hoodAngleSetpoint >= ShooterConstants.hoodMaximumAngle - 0.01;
            if (atMin || atMax) {
                setHoodVoltageOneMotor(0.0, 2);
            }
        } else {
            boolean pushingIntoMin = getHoodAnalogVoltage(2) < ShooterConstants.hoodMinimumAnalogVoltageCutoff
                && hood2VoltageSetpoint < 0;
            boolean pushingIntoMax = getHoodAnalogVoltage(2) > ShooterConstants.hood2MaximumAnalogVoltageCutoff
                && hood2VoltageSetpoint > 0;
            if (pushingIntoMin || pushingIntoMax) {
                setHoodVoltageOneMotor(0.0, 2);
            }
        }
    }

    /** Checks for disconnected motors and records sticky faults. */
    private void checkForFaultConditions() {
        recordedShooter1StickyFault = recordedShooter1StickyFault && RobotPreferences.isStickyFaultActive();
        recordedShooter2StickyFault = recordedShooter2StickyFault && RobotPreferences.isStickyFaultActive();
        recordedHood1StickyFault = recordedHood1StickyFault && RobotPreferences.isStickyFaultActive();
        recordedHood2StickyFault = recordedHood2StickyFault && RobotPreferences.isStickyFaultActive();

        if (!recordedShooter1StickyFault && !inputs.shooter1Connected) {
            RobotPreferences.recordStickyFaults("Shooter-Motor1");
            DataLogUtil.writeMessageEcho("STICKY FAULT Shooter: Shooter Motor 1 Missing");
            recordedShooter1StickyFault = true;
        }

        if (!recordedShooter2StickyFault && !inputs.shooter2Connected) {
            RobotPreferences.recordStickyFaults("Shooter-Motor2");
            DataLogUtil.writeMessageEcho("STICKY FAULT Shooter: Shooter Motor 2 Missing");
            recordedShooter2StickyFault = true;
        }

        if (!recordedHood1StickyFault && !inputs.hood1Connected) {
            RobotPreferences.recordStickyFaults("Hood-Motor1");
            DataLogUtil.writeMessageEcho("STICKY FAULT Shooter: Hood Motor 1 Missing");
            recordedHood1StickyFault = true;
        }

        if (!recordedHood2StickyFault && !inputs.hood2Connected) {
            RobotPreferences.recordStickyFaults("Hood-Motor2");
            DataLogUtil.writeMessageEcho("STICKY FAULT Shooter: Hood Motor 2 Missing");
            recordedHood2StickyFault = true;
        }
    }
}