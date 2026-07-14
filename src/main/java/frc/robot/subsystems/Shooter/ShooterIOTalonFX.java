package frc.robot.subsystems.Shooter;

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

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;


public class ShooterIOTalonFX implements ShooterIO {
    // Motor, configurator, and feedforward control objects
    private final TalonFX shooterMotor1 = new TalonFX(Ports.CANShooterMotor1);
    private final TalonFX shooterMotor2 = new TalonFX(Ports.CANShooterMotor2);
    private final TalonFXConfigurator shooterMotor1Configurator;
    private final TalonFXConfigurator shooterMotor2Configurator;
    private TalonFXConfiguration shooterMotorConfiguration;

    // Hood Linear Actuator Servo
    private final TalonFXS hoodMotor1 = new TalonFXS(Ports.CANHoodMotor1);
    private final TalonFXS hoodMotor2 = new TalonFXS(Ports.CANHoodMotor2);
    private final TalonFXSConfigurator hoodMotor1Configurator;
    private final TalonFXSConfigurator hoodMotor2Configurator;
    private TalonFXSConfiguration hoodMotorConfiguration;

    // Voltage and velocity controls
    private VoltageOut shooterVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
    private VelocityVoltage shooterVelocityControl = new VelocityVoltage(0).withSlot(0).withEnableFOC(false);

    private boolean shooterVelocityControlOn = false;  // true = velocity control is on, false = velocity control is off (e.g. percent output, position)
    private double shooterSetpointRPM;                 // Target velocity, in RPM (positive = forward, negative = reverse)
    
    private VoltageOut hood1VoltageControl = new VoltageOut(0.0).withEnableFOC(false);
    private VoltageOut hood2VoltageControl = new VoltageOut(0.0).withEnableFOC(false);
    private double hoodAngleSetpoint;
    private double hood1VoltageSetpoint;
    private double hood2VoltageSetpoint;
    private boolean hood1PositionControlOn = false;
    private boolean hood2PositionControlOn = false;
    
    
    public ShooterIOTalonFX() {
        // Get the motor configurator
        shooterMotor1Configurator = shooterMotor1.getConfigurator();
        shooterMotor2Configurator = shooterMotor2.getConfigurator();
        shooterMotorConfiguration = new TalonFXConfiguration();
        shooterMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorConfiguration.Voltage.PeakForwardVoltage = ShooterConstants.compensationVoltage;
        shooterMotorConfiguration.Voltage.PeakReverseVoltage = -ShooterConstants.compensationVoltage;
        shooterMotorConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;      // Time from 0 to full power in open loop, in seconds
        shooterMotorConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;  // Time from 0 to full power in closed loop, in seconds

        // If the current is above the supply current limit for the threshold time, the current is limited.
        // This is configured to prevent the breakers from tripping.
        shooterMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 50.0;      // Upper limit for the current, in amps
        shooterMotorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 25.0; // Lower limit for the current, in amps
        shooterMotorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.1;   // Threshold time, in seconds
        shooterMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Set PID coefficients
        shooterVelocityControl.Slot = 0;

        shooterVelocityControl.OverrideBrakeDurNeutral = false;
        shooterMotorConfiguration.Slot0.kP = ShooterConstants.ShooterkP;
        shooterMotorConfiguration.Slot0.kI = ShooterConstants.ShooterkI;
        shooterMotorConfiguration.Slot0.kD = ShooterConstants.ShooterkD;
        shooterMotorConfiguration.Slot0.kS = ShooterConstants.ShooterkS;
        shooterMotorConfiguration.Slot0.kV = ShooterConstants.ShooterkV;
        shooterMotorConfiguration.Slot0.kA = ShooterConstants.ShooterkA;

        hoodMotor1Configurator = hoodMotor1.getConfigurator();
        hoodMotor2Configurator = hoodMotor2.getConfigurator();
        hoodMotorConfiguration = new TalonFXSConfiguration();
        hoodMotorConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Brushed_DC;
        hoodMotorConfiguration.Commutation.BrushedMotorWiring = BrushedMotorWiringValue.Leads_A_and_B;
        hoodMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hoodMotorConfiguration.Voltage.PeakForwardVoltage = ShooterConstants.compensationVoltage;
        hoodMotorConfiguration.Voltage.PeakReverseVoltage = -ShooterConstants.compensationVoltage;
        hoodMotorConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;      // Time from 0 to full power in open loop, in seconds
        hoodMotorConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;  // Time from 0 to full power in closed loop, in seconds
        // hoodMotorConfiguration.SoftwareLimitSwitch.

        // If the current is above the supply current limit for the threshold time, the current is limited.
        // This is configured to prevent the breakers from tripping.
        hoodMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 60.0;      // Upper limit for the current, in amps
        hoodMotorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 25.0; // Lower limit for the current, in amps
        hoodMotorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.1;   // Threshold time, in seconds
        hoodMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Apply the configurations to the motor.
        // This is a blocking call and will wait up to 200ms to apply.
        shooterMotor1Configurator.apply(shooterMotorConfiguration);
        // Inversion configuration inverted by setControl for the follower motor
        // If motor 2 isn't being used as a follower, it will require a separate motor configuration
        shooterMotor2Configurator.apply(shooterMotorConfiguration);

        hoodMotor1Configurator.apply(hoodMotorConfiguration);
        hoodMotor2Configurator.apply(hoodMotorConfiguration);

        // Make motor2 follow motor1
        shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(),  MotorAlignmentValue.Opposed)); 
        // hoodMotor2.setControl(new Follower(hoodMotor1.getDeviceID(), MotorAlignmentValue.Aligned)); controlling both separately
        
        // Stop the motor
        stopShooterMotor();
        setHoodVoltageBothMotors(0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.TempCelsius1 = shooterMotor1.getDeviceTemp().getValueAsDouble();
        inputs.AppliedVolts1 = shooterMotor1.getMotorVoltage().getValueAsDouble();
        inputs.CurrentAmps1 = shooterMotor1.getStatorCurrent().getValueAsDouble();
        inputs.VelocityRotPerMin1 = shooterMotor1.getVelocity().getValueAsDouble() * ShooterConstants.gearRatio * 60.0;
        inputs.PositionRots1 = shooterMotor1.getPosition().getValueAsDouble() * ShooterConstants.gearRatio;
        inputs.SupplyVoltage1 = shooterMotor1.getSupplyCurrent().getValueAsDouble();
    
        inputs.TempCelsius2 = shooterMotor2.getDeviceTemp().getValueAsDouble();
        inputs.AppliedVolts2 = shooterMotor2.getMotorVoltage().getValueAsDouble();
        inputs.CurrentAmps2 = shooterMotor2.getStatorCurrent().getValueAsDouble();
        inputs.VelocityRotPerMin2 = shooterMotor2.getVelocity().getValueAsDouble() * ShooterConstants.gearRatio * 60.0;
        inputs.PositionRots2 = shooterMotor2.getPosition().getValueAsDouble() * ShooterConstants.gearRatio;
        inputs.SupplyVoltage2 = shooterMotor2.getSupplyCurrent().getValueAsDouble();
        
        inputs.HoodAngle = getHoodAngle();
        inputs.HoodAngle1 = getHoodAngle(1);
        inputs.HoodAnalogVoltage1 = hoodMotor1.getAnalogVoltage().getValueAsDouble();
        inputs.HoodAppliedVoltage1 = hoodMotor1.getMotorVoltage().getValueAsDouble();
        inputs.HoodSupplyVoltage1 = hoodMotor1.getSupplyVoltage().getValueAsDouble();
        inputs.HoodCurrent1 = hoodMotor1.getStatorCurrent().getValueAsDouble();
        inputs.HoodVoltageSetpoint1 = hood1VoltageSetpoint;
        inputs.HoodAngle2 = getHoodAngle(2);
        inputs.HoodAnalogVoltage2 = hoodMotor1.getAnalogVoltage().getValueAsDouble();
        inputs.HoodAppliedVoltage2 = hoodMotor1.getMotorVoltage().getValueAsDouble();
        inputs.HoodSupplyVoltage2 = hoodMotor1.getSupplyVoltage().getValueAsDouble();
        inputs.HoodCurrent2 = hoodMotor1.getStatorCurrent().getValueAsDouble();
        inputs.HoodVoltageSetpoint2 = hood2VoltageSetpoint;
    }

    @Override
    public void setShooterPercentOutput(double percent) {
        shooterVelocityControlOn = false;
        shooterSetpointRPM = 0.0;
        shooterMotor1.setControl(shooterVoltageControl.withOutput(percent * ShooterConstants.compensationVoltage));
    }

    @Override
    public void setShooterVelocity(double velocityRPM) {
    shooterVelocityControlOn = true;
    shooterSetpointRPM = velocityRPM;
    double motorRPS = (velocityRPM / 60.0) / ShooterConstants.gearRatio;
    shooterMotor1.setControl(
      shooterVelocityControl
        .withVelocity(motorRPS)
      );
    }

    @Override
    public void stopShooterMotor() {
    setShooterPercentOutput(0);
    }

    /**
   * Estimate current hood angle based on one hood motor's current length.
   * @param motorNumber hood motor # to estimate (1 = left, 2 = right)
   * @return estimated hood angle from specified motor's reading, in degrees
   */
    private double getHoodAngle(int motorNumber) {
        return Math.toDegrees(
        Math.acos(
            ((ShooterConstants.radius1 * ShooterConstants.radius1) + (ShooterConstants.radius2 * ShooterConstants.radius2)
            - (getHoodExtensionLength(motorNumber) * getHoodExtensionLength(motorNumber)))
            / (2.0 * ShooterConstants.radius1 * ShooterConstants.radius2)
        )
        ) - (54.8122837191 - ShooterConstants.hoodMinimumAngle);
    }
    /**
     * This should estimate current hood angle based on both hood motors' current length.
     * However, due to flaky left hood motor, this only returns the value from the right hood motor. 
     * @return estimated hood angle, in degrees
     */
    private double getHoodAngle() {
        return getHoodAngle(2);
        //HoodFix
        // return (getHoodAngle(1) + getHoodAngle(2)) / 2.0;
    }
    /**
   * Get estimated linear actuator length based on one hood motor's analog voltage, INCLUDING BASE OF ACTUATOR.
   * @param motorNumber hood # to estimate
   * @return estimated specified hood motor extension length, in inches
   */
    private double getHoodExtensionLength(int motorNumber) {
        return (motorNumber == 1) ? ShooterConstants.hood1VoltageToExtensionLengthSlope * hoodMotor1.getAnalogVoltage().getValueAsDouble() + ShooterConstants.hoodActuatorMinExtensionLength
    : ShooterConstants.hood2VoltageToExtensionLengthSlope * hoodMotor2.getAnalogVoltage().getValueAsDouble() + ShooterConstants.hoodActuatorMinExtensionLength;
    }
}