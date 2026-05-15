// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {

    // Shooter motors
    private final TalonFX shooterMotor1 = new TalonFX(Ports.CANShooterMotor1);
    private final TalonFX shooterMotor2 = new TalonFX(Ports.CANShooterMotor2);
    private final TalonFXConfigurator shooter1Configurator;
    private final TalonFXConfigurator shooter2Configurator;
    private TalonFXConfiguration shooterConfig;

    // Shooter signals
    private final StatusSignal<Temperature> shooter1Temp;
    private final StatusSignal<Current> shooter1Current;
    private final StatusSignal<Voltage> shooter1AppliedVolts;
    private final StatusSignal<Voltage> shooter1SupplyVoltage;
    private final StatusSignal<AngularVelocity> shooter1Velocity;
    private final StatusSignal<Angle> shooter1Position;

    private final StatusSignal<Temperature> shooter2Temp;
    private final StatusSignal<Current> shooter2Current;
    private final StatusSignal<Voltage> shooter2AppliedVolts;
    private final StatusSignal<Voltage> shooter2SupplyVoltage;
    private final StatusSignal<AngularVelocity> shooter2Velocity;
    private final StatusSignal<Angle> shooter2Position;

    // Hood motors
    private final TalonFXS hoodMotor1 = new TalonFXS(Ports.CANHoodMotor1);
    private final TalonFXS hoodMotor2 = new TalonFXS(Ports.CANHoodMotor2);
    private final TalonFXSConfigurator hood1Configurator;
    private final TalonFXSConfigurator hood2Configurator;
    private TalonFXSConfiguration hoodConfig;

    // Hood signals
    private final StatusSignal<Voltage> hood1AnalogVoltage;
    private final StatusSignal<Voltage> hood1AppliedVolts;
    private final StatusSignal<Voltage> hood1SupplyVoltage;
    private final StatusSignal<Current> hood1Current;

    private final StatusSignal<Voltage> hood2AnalogVoltage;
    private final StatusSignal<Voltage> hood2AppliedVolts;
    private final StatusSignal<Voltage> hood2SupplyVoltage;
    private final StatusSignal<Current> hood2Current;

    // Control requests
    private final VoltageOut shooterVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
    private final VelocityVoltage shooterVelocityControl = new VelocityVoltage(0.0).withSlot(0).withEnableFOC(false);
    private final VoltageOut hood1VoltageControl = new VoltageOut(0.0).withEnableFOC(false);
    private final VoltageOut hood2VoltageControl = new VoltageOut(0.0).withEnableFOC(false);

    public ShooterIOTalonFX() {
        shooter1Configurator = shooterMotor1.getConfigurator();
        shooter2Configurator = shooterMotor2.getConfigurator();

        shooter1Temp = shooterMotor1.getDeviceTemp();
        shooter1Current = shooterMotor1.getStatorCurrent();
        shooter1AppliedVolts = shooterMotor1.getMotorVoltage();
        shooter1SupplyVoltage = shooterMotor1.getSupplyVoltage();
        shooter1Velocity = shooterMotor1.getVelocity();
        shooter1Position = shooterMotor1.getPosition();

        shooter2Temp = shooterMotor2.getDeviceTemp();
        shooter2Current = shooterMotor2.getStatorCurrent();
        shooter2AppliedVolts = shooterMotor2.getMotorVoltage();
        shooter2SupplyVoltage = shooterMotor2.getSupplyVoltage();
        shooter2Velocity = shooterMotor2.getVelocity();
        shooter2Position = shooterMotor2.getPosition();

        hood1Configurator = hoodMotor1.getConfigurator();
        hood2Configurator = hoodMotor2.getConfigurator();

        hood1AnalogVoltage = hoodMotor1.getAnalogVoltage();
        hood1AppliedVolts = hoodMotor1.getMotorVoltage();
        hood1SupplyVoltage = hoodMotor1.getSupplyVoltage();
        hood1Current = hoodMotor1.getStatorCurrent();

        hood2AnalogVoltage = hoodMotor2.getAnalogVoltage();
        hood2AppliedVolts = hoodMotor2.getMotorVoltage();
        hood2SupplyVoltage = hoodMotor2.getSupplyVoltage();
        hood2Current = hoodMotor2.getStatorCurrent();

        buildShooterConfig();
        buildHoodConfig();
        configShooter();
    }


    // Config builders

    private void buildShooterConfig() {
        shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.Voltage.PeakForwardVoltage = ShooterConstants.compensationVoltage;
        shooterConfig.Voltage.PeakReverseVoltage = -ShooterConstants.compensationVoltage;
        shooterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
        shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        shooterConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
        shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;
        shooterConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterConfig.Slot0.kP = ShooterConstants.ShooterkP;
        shooterConfig.Slot0.kI = ShooterConstants.ShooterkI;
        shooterConfig.Slot0.kD = ShooterConstants.ShooterkD;
        shooterConfig.Slot0.kS = ShooterConstants.ShooterkS;
        shooterConfig.Slot0.kV = ShooterConstants.ShooterkV;
        shooterConfig.Slot0.kA = ShooterConstants.ShooterkA;
    }

    private void buildHoodConfig() {
        hoodConfig = new TalonFXSConfiguration();
        hoodConfig.Commutation.MotorArrangement = MotorArrangementValue.Brushed_DC;
        hoodConfig.Commutation.BrushedMotorWiring = BrushedMotorWiringValue.Leads_A_and_B;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.Voltage.PeakForwardVoltage = ShooterConstants.compensationVoltage;
        hoodConfig.Voltage.PeakReverseVoltage = -ShooterConstants.compensationVoltage;
        hoodConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
        hoodConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        hoodConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;
        hoodConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    }


    // ShooterIO

    @Override
    public void configShooter() {
        shooter1Configurator.apply(shooterConfig);
        shooter2Configurator.apply(shooterConfig);
        hood1Configurator.apply(hoodConfig);
        hood2Configurator.apply(hoodConfig);

        // Motor 2 opposes motor 1 (mounted facing opposite direction)
        shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooter1Connected = BaseStatusSignal.refreshAll(
            shooter1Temp, shooter1Current, shooter1AppliedVolts,
            shooter1SupplyVoltage, shooter1Velocity, shooter1Position).isOK();

        inputs.shooter2Connected = BaseStatusSignal.refreshAll(
            shooter2Temp, shooter2Current, shooter2AppliedVolts,
            shooter2SupplyVoltage, shooter2Velocity, shooter2Position).isOK();

        inputs.hood1Connected = BaseStatusSignal.refreshAll(
            hood1AnalogVoltage, hood1AppliedVolts, hood1SupplyVoltage, hood1Current).isOK();

        inputs.hood2Connected = BaseStatusSignal.refreshAll(
            hood2AnalogVoltage, hood2AppliedVolts, hood2SupplyVoltage, hood2Current).isOK();

        inputs.shooter1TempCelsius = shooter1Temp.getValueAsDouble();
        inputs.shooter1CurrentAmps = shooter1Current.getValueAsDouble();
        inputs.shooter1AppliedVolts = shooter1AppliedVolts.getValueAsDouble();
        inputs.shooter1SupplyVoltage = shooter1SupplyVoltage.getValueAsDouble();
        inputs.shooter1VelocityRPS = shooter1Velocity.getValueAsDouble();
        inputs.shooter1PositionRotations = shooter1Position.getValueAsDouble();

        inputs.shooter2TempCelsius = shooter2Temp.getValueAsDouble();
        inputs.shooter2CurrentAmps = shooter2Current.getValueAsDouble();
        inputs.shooter2AppliedVolts = shooter2AppliedVolts.getValueAsDouble();
        inputs.shooter2SupplyVoltage = shooter2SupplyVoltage.getValueAsDouble();
        inputs.shooter2VelocityRPS = shooter2Velocity.getValueAsDouble();
        inputs.shooter2PositionRotations = shooter2Position.getValueAsDouble();

        inputs.hood1AnalogVoltage = hood1AnalogVoltage.getValueAsDouble();
        inputs.hood1AppliedVolts = hood1AppliedVolts.getValueAsDouble();
        inputs.hood1SupplyVoltage = hood1SupplyVoltage.getValueAsDouble();
        inputs.hood1CurrentAmps = hood1Current.getValueAsDouble();

        inputs.hood2AnalogVoltage = hood2AnalogVoltage.getValueAsDouble();
        inputs.hood2AppliedVolts = hood2AppliedVolts.getValueAsDouble();
        inputs.hood2SupplyVoltage = hood2SupplyVoltage.getValueAsDouble();
        inputs.hood2CurrentAmps = hood2Current.getValueAsDouble();
    }

    @Override
    public void setShooterVelocity(double velocityRPS) {
        shooterMotor1.setControl(shooterVelocityControl.withVelocity(velocityRPS));
    }

    @Override
    public void setShooterVoltage(double volts) {
        shooterMotor1.setControl(shooterVoltageControl.withOutput(volts));
    }

    @Override
    public void setHood1Voltage(double volts) {
        hoodMotor1.setControl(hood1VoltageControl.withOutput(volts));
    }

    @Override
    public void setHood2Voltage(double volts) {
        hoodMotor2.setControl(hood2VoltageControl.withOutput(volts));
    }
}