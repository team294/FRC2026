// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // Shooter motor 1
        public boolean shooter1Connected = false;
        public double shooter1TempCelsius = 0.0;
        public double shooter1CurrentAmps = 0.0;
        public double shooter1AppliedVolts = 0.0;
        public double shooter1SupplyVoltage = 0.0;
        public double shooter1VelocityRPS = 0.0;
        public double shooter1PositionRotations = 0.0;

        // Shooter motor 2
        public boolean shooter2Connected = false;
        public double shooter2TempCelsius = 0.0;
        public double shooter2CurrentAmps = 0.0;
        public double shooter2AppliedVolts = 0.0;
        public double shooter2SupplyVoltage = 0.0;
        public double shooter2VelocityRPS = 0.0;
        public double shooter2PositionRotations = 0.0;

        // Hood motor 1
        public boolean hood1Connected = false;
        public double hood1AnalogVoltage = 0.0;
        public double hood1AppliedVolts = 0.0;
        public double hood1SupplyVoltage = 0.0;
        public double hood1CurrentAmps = 0.0;

        // Hood motor 2
        public boolean hood2Connected = false;
        public double hood2AnalogVoltage = 0.0;
        public double hood2AppliedVolts = 0.0;
        public double hood2SupplyVoltage = 0.0;
        public double hood2CurrentAmps = 0.0;
    }

    /** Updates the logged inputs. Must be called periodically. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Run shooter motors at specified velocity in RPS (motor shaft). Motor 2 follows motor 1. */
    public default void setShooterVelocity(double velocityRPS) {}

    /** Run shooter motors at specified voltage. Motor 2 follows motor 1. */
    public default void setShooterVoltage(double volts) {}

    /** Run hood motor 1 at specified voltage. */
    public default void setHood1Voltage(double volts) {}

    /** Run hood motor 2 at specified voltage. */
    public default void setHood2Voltage(double volts) {}

    /**
     * Configures all four motors. Should only be called from the constructor or to recover from a brownout.
     * <p><b>NOTE:</b> includes blocking CAN calls that will delay robot code.
     */
    public default void configShooter() {}
}