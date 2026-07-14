package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSub extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double hoodAngleSetpoint;
    private boolean hood1PositionControlOn = false;
    private boolean hood2PositionControlOn = false;
  
    private PIDController hood1PidController = new PIDController(ShooterConstants.hood1kP, ShooterConstants.hood1kI, ShooterConstants.hood1kD);
    private PIDController hood2PidController = new PIDController(ShooterConstants.hood2kP, ShooterConstants.hood2kI, ShooterConstants.hood2kD);

    public ShooterSub(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        // Stopping hood motion when at min/max extensions to prevent motor from dying
        if (hood1PositionControlOn) {
            if ((inputs.HoodAnalogVoltage1 < ShooterConstants.hoodMinimumAnalogVoltageCutoff && hoodAngleSetpoint <= ShooterConstants.hoodMinimumAngle + 0.01) 
            || (inputs.HoodAnalogVoltage1 > ShooterConstants.hood1MaximumAnalogVoltageCutoff && hoodAngleSetpoint >= ShooterConstants.hoodMaximumAngle - 0.01)) {

            io.setHoodVoltageOneMotor(0, 1);
        }
        } else {
            if ((inputs.HoodAnalogVoltage1 < ShooterConstants.hoodMinimumAnalogVoltageCutoff && inputs.HoodVoltageSetpoint1 < 0) 
            || (inputs.HoodAnalogVoltage1 > ShooterConstants.hood1MaximumAnalogVoltageCutoff && inputs.HoodVoltageSetpoint1 > 0)) {

            io.setHoodVoltageOneMotor(0, 1);
        }
        }

        if (hood2PositionControlOn) {
            if ((inputs.HoodAnalogVoltage2 < ShooterConstants.hoodMinimumAnalogVoltageCutoff && hoodAngleSetpoint <= ShooterConstants.hoodMinimumAngle + 0.01)
            || (inputs.HoodAnalogVoltage2 > ShooterConstants.hood2MaximumAnalogVoltageCutoff && hoodAngleSetpoint >= ShooterConstants.hoodMaximumAngle - 0.01)) {

            io.setHoodVoltageOneMotor(0, 2);
        }
        } else {
            if ((inputs.HoodAnalogVoltage2 < ShooterConstants.hoodMinimumAnalogVoltageCutoff && inputs.HoodVoltageSetpoint2 < 0) 
            || (inputs.HoodAnalogVoltage2 > ShooterConstants.hood2MaximumAnalogVoltageCutoff && inputs.HoodVoltageSetpoint2 > 0)) {

            io.setHoodVoltageOneMotor(0, 2);
        }
        }
    }
}
