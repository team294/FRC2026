// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {

    // Shooter flywheel simulation.
    // Both motors are modeled independently with the same inertia/gear ratio;
    // motor 2 receives the same commanded voltage as motor 1, mirroring the hardware Follower.
    private final DCMotorSim shooter1Sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.003, // moment of inertia (kg·m²) — tune to match real shooter spin-up time
            ShooterConstants.gearRatio),
        DCMotor.getKrakenX60(1));

    private final DCMotorSim shooter2Sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.003,
            ShooterConstants.gearRatio),
        DCMotor.getKrakenX60(1));

    // Shooter velocity closed-loop.
    // Replaces the TalonFX onboard VelocityVoltage loop, which has no effect in sim.
    private final PIDController shooterPID = new PIDController(
        ShooterConstants.ShooterkP,
        ShooterConstants.ShooterkI,
        ShooterConstants.ShooterkD);

    // Shooter commanded state
    private double shooter1AppliedVolts = 0.0;
    private double shooter2AppliedVolts = 0.0;
    private double shooterVelocitySetpointRPS = 0.0;
    private boolean shooterClosedLoop = false;

    // Hood simulation.
    // The hood linear actuators are brushed DC motors. Rather than simulating the full
    // mechanical linkage, we integrate extension length directly from the applied voltage,
    // then back-calculate the analog voltage that the real sensor would read.
    // extensionRateInchesPerVoltSecond is an approximation — tune to match real actuator speed.
    private static final double EXTENSION_RATE_INCHES_PER_VOLT_SECOND = 0.08;

    private double hood1ExtensionInches = ShooterConstants.hoodActuatorMinExtensionLength;
    private double hood2ExtensionInches = ShooterConstants.hoodActuatorMinExtensionLength;
    private double hood1AppliedVolts = 0.0;
    private double hood2AppliedVolts = 0.0;

    private static final double LOOP_PERIOD_SECS = 0.02;

    public ShooterIOSim() {}


    // ShooterIO

    /**
     * Steps the simulation forward one loop period and populates inputs.
     * Should be called every periodic loop.
     */
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Run shooter velocity closed-loop if active
        if (shooterClosedLoop) {
            double pidOutput = shooterPID.calculate(shooter1Sim.getAngularVelocityRadPerSec(),
                shooterVelocitySetpointRPS * 2.0 * Math.PI);
            // kS and kV feedforward, mirroring the TalonFX Slot0 gains
            double ff = ShooterConstants.ShooterkS * Math.signum(shooterVelocitySetpointRPS)
                + ShooterConstants.ShooterkV * shooterVelocitySetpointRPS * 2.0 * Math.PI;
            shooter1AppliedVolts = MathUtil.clamp(ff + pidOutput, -12.0, 12.0);
            shooter2AppliedVolts = shooter1AppliedVolts; // follower
        }

        // Step shooter sims
        shooter1Sim.setInputVoltage(shooter1AppliedVolts);
        shooter2Sim.setInputVoltage(shooter2AppliedVolts);
        shooter1Sim.update(LOOP_PERIOD_SECS);
        shooter2Sim.update(LOOP_PERIOD_SECS);

        // Integrate hood extension lengths from applied voltage
        // Clamp to actuator physical limits
        hood1ExtensionInches = MathUtil.clamp(
            hood1ExtensionInches + hood1AppliedVolts * EXTENSION_RATE_INCHES_PER_VOLT_SECOND * LOOP_PERIOD_SECS,
            ShooterConstants.hoodActuatorMinExtensionLength,
            ShooterConstants.hoodActuatorMaxExtensionLength);
        hood2ExtensionInches = MathUtil.clamp(
            hood2ExtensionInches + hood2AppliedVolts * EXTENSION_RATE_INCHES_PER_VOLT_SECOND * LOOP_PERIOD_SECS,
            ShooterConstants.hoodActuatorMinExtensionLength,
            ShooterConstants.hoodActuatorMaxExtensionLength);

        // All simulated devices are always "connected"
        inputs.shooter1Connected = true;
        inputs.shooter2Connected = true;
        inputs.hood1Connected = true;
        inputs.hood2Connected = true;

        // Shooter motor 1
        inputs.shooter1VelocityRPS = shooter1Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        inputs.shooter1PositionRotations = shooter1Sim.getAngularPositionRotations();
        inputs.shooter1AppliedVolts = shooter1AppliedVolts;
        inputs.shooter1CurrentAmps = Math.abs(shooter1Sim.getCurrentDrawAmps());
        inputs.shooter1SupplyVoltage = 12.0;
        inputs.shooter1TempCelsius = 0.0;

        // Shooter motor 2
        inputs.shooter2VelocityRPS = shooter2Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        inputs.shooter2PositionRotations = shooter2Sim.getAngularPositionRotations();
        inputs.shooter2AppliedVolts = shooter2AppliedVolts;
        inputs.shooter2CurrentAmps = Math.abs(shooter2Sim.getCurrentDrawAmps());
        inputs.shooter2SupplyVoltage = 12.0;
        inputs.shooter2TempCelsius = 0.0;

        // Hood motor 1 — back-calculate analog voltage from simulated extension length
        inputs.hood1AnalogVoltage = (hood1ExtensionInches - ShooterConstants.hoodActuatorMinExtensionLength)
            / ShooterConstants.hood1VoltageToExtensionLengthSlope;
        inputs.hood1AppliedVolts = hood1AppliedVolts;
        inputs.hood1SupplyVoltage = 12.0;
        inputs.hood1CurrentAmps = 0.0;

        // Hood motor 2
        inputs.hood2AnalogVoltage = (hood2ExtensionInches - ShooterConstants.hoodActuatorMinExtensionLength)
            / ShooterConstants.hood2VoltageToExtensionLengthSlope;
        inputs.hood2AppliedVolts = hood2AppliedVolts;
        inputs.hood2SupplyVoltage = 12.0;
        inputs.hood2CurrentAmps = 0.0;
    }

    @Override
    public void setShooterVelocity(double velocityRPS) {
        shooterClosedLoop = true;
        shooterVelocitySetpointRPS = velocityRPS;
    }

    @Override
    public void setShooterVoltage(double volts) {
        shooterClosedLoop = false;
        shooter1AppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        shooter2AppliedVolts = shooter1AppliedVolts; // follower
    }

    @Override
    public void setHood1Voltage(double volts) {
        hood1AppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setHood2Voltage(double volts) {
        hood2AppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    // configShooter has no effect in simulation (no hardware to configure)
    @Override
    public void configShooter() {}
}