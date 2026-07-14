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

        public double HoodAngle;
        public double HoodAngle1;
        public double HoodAnalogVoltage1;
        public double HoodAppliedVoltage1;
        public double HoodSupplyVoltage1;
        public double HoodCurrent1;
        public double HoodVoltageSetpoint1;
        public double HoodAngle2;
        public double HoodAnalogVoltage2;
        public double HoodAppliedVoltage2;
        public double HoodSupplyVoltage2;
        public double HoodCurrent2;
        public double HoodVoltageSetpoint2;

        // Sticky Faults
        public boolean recordedShooterMotor1MissingStickyFault;
        public boolean recordedShooterMotor2MissingStickyFault;

        public boolean recordedHoodMotor1MissingStickyFault;
        public boolean recordedHoodMotor2MissingStickyFault;

    }
    default void updateInputs(ShooterIOInputs inputs) {}
    /**
     * Sets the percent output of the Shooter motor using voltage compensation.
     * @param percent output percent, -1.0 to 1.0 (positive = shoot direction, negative = reverse)
     */
    default public void setShooterPercentOutput(double percent) {}
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
}

