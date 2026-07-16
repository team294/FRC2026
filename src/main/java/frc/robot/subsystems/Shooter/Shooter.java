package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private String subsystemName;
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

    private double hoodAngleSetpoint;
    private boolean hood1PositionControlOn = false;
    private boolean hood2PositionControlOn = false;
  
    private PIDController hood1PidController = new PIDController(ShooterConstants.hood1kP, ShooterConstants.hood1kI, ShooterConstants.hood1kD);
    private PIDController hood2PidController = new PIDController(ShooterConstants.hood2kP, ShooterConstants.hood2kI, ShooterConstants.hood2kD);
    

    public Shooter(String subsystemName, ShooterIO io) {
        this.subsystemName = subsystemName;
        this.io = io;
        hood1PidController.reset();
        hood2PidController.reset();
        hood1PidController.setTolerance(ShooterConstants.hoodPIDErrorTolerance);
        hood2PidController.setTolerance(ShooterConstants.hoodPIDErrorTolerance);
    }

    @Override
    public void periodic() {
        io.checkForFaultConditions();
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        // Stopping hood motion when at min/max extensions to prevent motor from dying
        if (hood1PositionControlOn) {
            if ((hoodInputs.AnalogVoltage1 < ShooterConstants.hoodMinimumAnalogVoltageCutoff && hoodAngleSetpoint <= ShooterConstants.hoodMinimumAngle + 0.01) 
            || (hoodInputs.AnalogVoltage1 > ShooterConstants.hood1MaximumAnalogVoltageCutoff && hoodAngleSetpoint >= ShooterConstants.hoodMaximumAngle - 0.01)) {

            io.setHoodVoltageOneMotor(0, 1);
            }
        } else {
            if ((hoodInputs.AnalogVoltage1 < ShooterConstants.hoodMinimumAnalogVoltageCutoff && hoodInputs.VoltageSetpoint1 < 0) 
            || (hoodInputs.AnalogVoltage1 > ShooterConstants.hood1MaximumAnalogVoltageCutoff && hoodInputs.VoltageSetpoint1 > 0)) {

            io.setHoodVoltageOneMotor(0, 1);
            }
        }

        if (hood2PositionControlOn) {
            if ((hoodInputs.AnalogVoltage2 < ShooterConstants.hoodMinimumAnalogVoltageCutoff && hoodAngleSetpoint <= ShooterConstants.hoodMinimumAngle + 0.01)
            || (hoodInputs.AnalogVoltage2 > ShooterConstants.hood2MaximumAnalogVoltageCutoff && hoodAngleSetpoint >= ShooterConstants.hoodMaximumAngle - 0.01)) {

            io.setHoodVoltageOneMotor(0, 2);
            }
        } else {
            if ((hoodInputs.AnalogVoltage2 < ShooterConstants.hoodMinimumAnalogVoltageCutoff && hoodInputs.VoltageSetpoint2 < 0) 
            || (hoodInputs.AnalogVoltage2 > ShooterConstants.hood2MaximumAnalogVoltageCutoff && hoodInputs.VoltageSetpoint2 > 0)) {

            io.setHoodVoltageOneMotor(0, 2);
            }
        }
        Logger.recordOutput("Hood/PositionControlOn", hood2PositionControlOn);
    }

    /**
     * Returns if shooter is within tolerance from the most recent goal velocity
     * @return true = in tolerance, false = not at goal velocity.
    */
    public boolean isShooterAtGoalVelocity() {
        return Math.abs(getShooterVelocityPIDError()) < ShooterConstants.shooterVelocityErrorTolerance && (inputs.VelocitySet != 0.0);
    }
    /**
     * Gets the error of the PID for spindexerMotor velocity control.
     * @return error, in RPM (positive = measured speed is too fast, negative = too slow)
    */
    public double getShooterVelocityPIDError() {
        return getShooterVelocity() - inputs.VelocitySet;
    }

    /**
     * Gets the velocity of the shooter wheel, averaging the two motors.
     * @return wheel velocity, in RPM
     */
    public double getShooterVelocity() {
        return (inputs.VelocityRotPerMin1 + inputs.VelocityRotPerMin2) / 2.0;
    }
    

    /**
   * Move both motors with simple PID controller.
   * <p><b>Note:</b> This method needs to be called every execute() cycle to continue using PID control.
   * @param angle angle to set in degrees
   */
    public void setHoodAnglePID(double angle) {
        double safeAngle = MathUtil.clamp(angle, ShooterConstants.hoodMinimumAngle, ShooterConstants.hoodMaximumAngle);
        hoodAngleSetpoint = safeAngle;
        hood1PositionControlOn = true;
        hood2PositionControlOn = true;
    
        if ( Math.abs(hoodInputs.Angle - hoodAngleSetpoint) < ShooterConstants.hoodAngleErrorTolerance/2.0 ) {
        // If hood is very close to target position, then turn off motor.
        io.setHoodSpeed(1, 0.0);
        io.setHoodSpeed(2, 0.0);
        } else {
            // hoodMotor1.set(hood1PidController.calculate(getHoodAngle(1), hoodAngleSetpoint)); 
            // hoodMotor2.set(hood2PidController.calculate(getHoodAngle(2), hoodAngleSetpoint)); 
            //HoodFix
            // Due to error on the left (#1) hood analog sensor, calculate PID based on right hood motor
            // and set motor #1 as a follower with a fixed calibration ratio for motor/friction differences.
            double hoodPct = hood2PidController.calculate(hoodInputs.Angle2, hoodAngleSetpoint);
            io.setHoodSpeed(1, hoodPct * ShooterConstants.hoodMotorRatio2to1);
            io.setHoodSpeed(2 , hoodPct);
        }
    }
    /**
     * Returns the hood angle setpoint in degrees
     * @return the hood angle setpoint in degrees
    */
    public double getHoodAngleSetpoint() {
        return hoodAngleSetpoint;
    }
    /**
     * Returns if hood is within tolerance from the most recent goal angle
     * @return true = in tolerance, false = not at goal angle.
     */
    public boolean isHoodAtGoalAngle() {
        return Math.abs(hoodInputs.Angle - hoodAngleSetpoint) < ShooterConstants.hoodAngleErrorTolerance;
    }

    /**
     * Returns if hood is within tolerance of trench configuration
     * @return true = in tolerance, false = not in trench configuration.
     */
    public boolean isHoodSafeForTrench() {
        return Math.abs(hoodInputs.Angle1 - ShooterConstants.hoodMinimumAngle) < ShooterConstants.hoodAngleErrorTolerance
            && Math.abs(hoodInputs.Angle2 - ShooterConstants.hoodMinimumAngle) < ShooterConstants.hoodAngleErrorTolerance;
    }

    // *****  Information methods *****

    /**
     * Gets the name of the subsystem.
     * @return the subsystem name
     */
    public String getName() {
        return subsystemName;
    }

    // **** IO Wrapper Methods *****
    /**
     * Sets the percent output of the Shooter motor using voltage compensation.
     * @param percent output percent, -1.0 to 1.0 (positive = shoot direction, negative = reverse)
     */
    public void setShooterPercentOutput(double percent) {
        io.setShooterPercentOutput(percent);
    }
    /**
     * Sets the velocity of the Shooter wheel using voltage compensation. 
     * @param velocityRPM the velocity, in rpm, to set the wheel to (positive = shoot, negative = reverse)
     */
    public void setShooterVelocity(double velocityRPM) {
        io.setShooterVelocity(velocityRPM);
    }
    /**
     * Stops the Shooter motor.
     */
    public void stopShooterMotor() {
        io.stopShooterMotor();
    }
    /**
     * Common interface for setting the speed of a motor controller.
     * @param speed The speed to set. Value should be between -1.0 and 1.0. */
    public void setHoodSpeed(int motorNumber, double speed) {
        io.setHoodSpeed(motorNumber, speed);
    }

    /**
     * Set the hood voltage to BOTH linear actuators
     * @param voltage in volts [-12, 12]; positive is up and negative is down
     */
    public void setHoodVoltageBothMotors(double voltage) {
        // Now need to set flag at subsystem level
        hood1PositionControlOn = false;
        hood2PositionControlOn = false;
        io.setHoodVoltageBothMotors(voltage);
    }

    /**
    * Set the hood voltage to ONE linear actuator 
    * @param voltage in volts [-12, 12]; positive is up and negative is down
    */
    public void setHoodVoltageOneMotor(double voltage, int motorNumber) {
        // Set the flag to correct motor at subsystem level
        if (motorNumber == 1) {
            hood1PositionControlOn = false;
        } else {
            hood2PositionControlOn = false;
        }
        io.setHoodVoltageOneMotor(voltage, motorNumber);
    }
}
