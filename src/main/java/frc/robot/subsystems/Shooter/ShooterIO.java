package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double TempCelsius1 = 0.0;
        public double CurrentAmps1 = 0.0;
        public double VelocityRotPerMin1 = 0.0;
        public double PositionRots1 = 0.0;
        public double AppliedVolts1 = 0.0;
        public double SupplyVoltage1 = 0.0;
        public double TempCelsius2 = 0.0;
        public double CurrentAmps2 = 0.0;
        public double VelocityRotPerMin2 = 0.0;
        public double PositionRots2 = 0.0;
        public double AppliedVolts2 = 0.0;
        public double SupplyVoltage2 = 0.0;
        public double VelocitySet = 0.0;
        public boolean recordedShooterMotor1MissingStickyFault = false;
        public boolean recordedShooterMotor2MissingStickyFault = false;
    }
    @AutoLog
    class HoodIOInputs {
        public double Angle = 0.0;
        public double Angle1 = 0.0;
        public double AnalogVoltage1 = 0.0;
        public double AppliedVoltage1 = 0.0;
        public double SupplyVoltage1 = 0.0;
        public double Current1 = 0.0;
        public double VoltageSetpoint1 = 0.0;
        public double Angle2 = 0.0;
        public double AnalogVoltage2 = 0.0;
        public double AppliedVoltage2 = 0.0;
        public double SupplyVoltage2 = 0.0;
        public double Current2 = 0.0;
        public double VoltageSetpoint2 = 0.0;
        public boolean recordedHoodMotor1MissingStickyFault = false;
        public boolean recordedHoodMotor2MissingStickyFault = false;
    }
    default void updateInputs(ShooterIOInputs inputs) {}
    default void updateHoodInputs(HoodIOInputs inputs) {}
    /**
     * Sets the percent output of the Shooter motor using voltage compensation.
     * @param percent output percent, -1.0 to 1.0 (positive = shoot direction, negative = reverse)
     */
    default public void setShooterPercentOutput(double percent) {}
    /**
     * Checks for fault conditions and records stick faults if any exist.
     */
    default public  void checkForFaultConditions() {}
    /**
     * Sets the velocity of the Shooter wheel using voltage compensation. 
     * @param velocityRPM the velocity, in rpm, to set the wheel to (positive = shoot, negative = reverse)
     */
    default public void setShooterVelocity(double velocityRPM) {}
    /**
     * Stops the Shooter motor.
     */
    default public void stopShooterMotor() {}
    /**
     * Set the hood voltage to BOTH linear actuators 
     * @param voltage in volts [-12, 12]; positive is up and negative is down
     */
    default public void setHoodVoltageBothMotors(double voltage) {}
    /**
     * Set the hood voltage to ONE linear actuator 
     * @param voltage in volts [-12, 12]; positive is up and negative is down
     */
    default public void setHoodVoltageOneMotor(double voltage, int motorNumber) {}
    /**
     * Common interface for setting the speed of a motor controller.
     * @param speed The speed to set. Value should be between -1.0 and 1.0. */
    default public void setHoodSpeed(int motorNumber, double speed) {}
}

