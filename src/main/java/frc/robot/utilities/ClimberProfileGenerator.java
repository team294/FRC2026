package frc.robot.utilities;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberProfileGenerator {
  Climber climber;

  private boolean profileEnabled = false;

  private double currentMPDistance;   // Distance that should have been traveled in the current motion profile (always positive)
  private double targetMPDistance;    // Total distance to be traveled in the current motion profile (always positive)

  private double initialPosition;     // Initial position from the bottom of the climber, in inches
  private double finalPosition;       // Final position from the bottom of the climber, in inches

  private double maxVelocity = 1.0;  // Max velocity, in inches/second TODO CALIBRATE
  private double currentMPVelocity;   // Velocity that it should be at in the current motion profile

  private double maxAcceleration = 1.0;                       // Max acceleration, in inches/second^2 TODO CALIBRATE
  private double stoppingAcceleration = 0.01 * maxAcceleration;  // Limit the stopping acceleration to x% of the max acceleration TODO CALIBRATE
  private double currentMPAcceleration;                         // Acceleration that it should be at in the current motion profile
  private boolean approachingTarget = false;                    // true = close enough to target to be decelerating, false = not close enough

  private double MPdt = 0.02;   // delta T (time) in seconds forecasted between future calls to updateProfileCalcs
  private double measuredDt;    // delta T (time) in seconds measured between this call and the prior call to updateProfileCalcs

  private double directionSign; // +1 if finalPosition > initialPosition, -1 if not

  private long startTime, lastTime;

  private double prevError, error, intError, lastMPVelocity;

  double percentPowerFF = 0;
  double percentPowerFB = 0;

  // TODO CALIBRATE
  private double kFF = 0.0 / ClimberConstants.compensationVoltage;     // In pct-output
  private double kSu = 0.0 / ClimberConstants.compensationVoltage;     // In pct-output
  private double kVu = 0.0 / ClimberConstants.compensationVoltage;     // In (pct-output) / (in/s)
  private double kAu = 0.0 / ClimberConstants.compensationVoltage;    // In (pct-output) / (in/s^2)
  private double kPu = 0.0;        // In (pct-output) / (in)
  private double kIu = 0.0;
  private double kDu = 0.0;
  private double kSd = 0.0 / ClimberConstants.compensationVoltage;     // In pct-output
  private double kVd = 0.0 / ClimberConstants.compensationVoltage;     // In (pct-output) / (in/s)
  private double kAd = 0.0 / ClimberConstants.compensationVoltage;    // In (pct-output) / (in/s^2)
  private double kPd = 0.0;       // In (pct-output) / (in)
  private double kId = 0.0;
  private double kDd = 0.0;

  // Calcs for discrete timestep feedforward
  private double Adu = Math.exp( -kVu/kAu * MPdt);
  private double Bdu = kVu / (1 - Adu);
  private double Add = Math.exp( -kVd/kAd * MPdt);
  private double Bdd = kVd / (1 - Add);

  // Variables for DataLogging
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogTargetPos = new DoubleLogEntry(log, "/ClimberProfile/TargetPos");
  private final DoubleLogEntry dLogTime = new DoubleLogEntry(log, "/ClimberProfile/Time");
  private final DoubleLogEntry dLogDt = new DoubleLogEntry(log, "/ClimberProfile/dt");
  private final DoubleLogEntry dLogMPPos = new DoubleLogEntry(log, "/ClimberProfile/MPPos");
  private final DoubleLogEntry dLogActualPos = new DoubleLogEntry(log, "/ClimberProfile/ActualPos");
  private final DoubleLogEntry dLogMPVel = new DoubleLogEntry(log, "/ClimberProfile/MPVel");
  private final DoubleLogEntry dLogActualVel = new DoubleLogEntry(log, "/ClimberProfile/ActualVel");
  private final DoubleLogEntry dLogMPAccel = new DoubleLogEntry(log, "/ClimberProfile/MPAccel");
  private final DoubleLogEntry dLogPctFF = new DoubleLogEntry(log, "/ClimberProfile/PctFF");
  private final DoubleLogEntry dLogPctFB = new DoubleLogEntry(log, "/ClimberProfile/PctFB");


  /**
   * Creates a new profile generator in disabled mode.
   * @param climber Climber subsystem
   */
  public ClimberProfileGenerator(Climber climber) {
    disableProfileControl();
    
    this.climber = climber;

    // Prime the DataLog to reduce delay when first enabling the robot
    updateClimberProfileLog(true);
  }

  /**
   * Disables the motion profile's control of motors.
   */
  public void disableProfileControl() {
    profileEnabled = false;
  }

  /**
   * Sets the target position for climber, using motion profile movement. This enables the profiler
   * to take control of the climber, so only call it if the encoders are working (check interlocks first).
   * @param pos target, in inches (use ClimberConstants.ClimberPosition)
   */
  public void setProfileTarget(double pos) {
    profileEnabled = true;
    finalPosition = pos;
    initialPosition = climber.getClimberPosition();
    intError = 0;   // Clear integrated error
    prevError = 0;  // Clear previous error
    approachingTarget = false;

    // Save the starting time
    startTime = System.currentTimeMillis();
    lastTime = startTime;

    // If target position = initial position, then set the target to 
    // the min distance needed to stop the current climber motion.
    if (finalPosition == initialPosition) {
      double curVel = climber.getClimberVelocity();
      finalPosition += Math.signum(curVel) * 0.5 * curVel * curVel / maxAcceleration;
    }

    SmartDashboard.putNumber("ClimberInitPos", initialPosition);
    SmartDashboard.putNumber("ClimberTarget", finalPosition);

    currentMPDistance = 0;
    targetMPDistance = Math.abs(finalPosition - initialPosition);
    directionSign = Math.signum(finalPosition - initialPosition);

    // Seed the profile with the current velocity, in case the climber is already moving
    currentMPVelocity = climber.getClimberVelocity() * directionSign;

    DataLogUtil.writeMessage("ClimberProfile:  New Profile, InitPos =", initialPosition , ", Target =", finalPosition);
  }

  /**
   * Call this method once per scheduler cycle. This method calculates the distance that the climber should 
   * have traveled at this point in time, per the motion profile. It also calculates velocity in inches/second.
   */

  public void updateProfileCalcs() {
    long currentTime = System.currentTimeMillis();
    measuredDt = ((double) (currentTime - lastTime)) / 1000.0;
    lastTime = currentTime;
    lastMPVelocity = currentMPVelocity;

    if (currentMPDistance < targetMPDistance) { 
      // The profile has not reached its target, so update the profile velocity and acceleration
      // Do not continue calculating after the profile *should have* reached its target

      double stoppingDistance = 0.5 * currentMPVelocity * currentMPVelocity / stoppingAcceleration;

      // The profile *should be* close enough to the target to be decelerating, so update the acceleration
      if (currentMPVelocity >= 0 && (approachingTarget || (targetMPDistance - currentMPDistance) < stoppingDistance) ) {
        approachingTarget = true;
        // If the climber is too close to the target, then it may need to decelerate faster than stoppingAcceleration.
        // If not, then just use stoppingAcceleration.
        currentMPAcceleration = Math.min(-stoppingAcceleration, -0.5 * currentMPVelocity * currentMPVelocity / (targetMPDistance - currentMPDistance));
      }

      // The profile has a target velocity less than the maximum, so accelerate
      else if (currentMPVelocity < maxVelocity) {
        currentMPAcceleration = maxAcceleration;
      }

      // The profile has a target velocity at the maximum, so do not accelerate
      else {
        currentMPAcceleration = 0.0;
      }

      // Calculate the target velocity, capped by the maximum
      currentMPVelocity = currentMPVelocity + currentMPAcceleration * MPdt;
      if (currentMPVelocity > maxVelocity) {
        currentMPVelocity = maxVelocity;
      }
      if (approachingTarget && currentMPVelocity<0.0) {
        // Do not set the acceleration too negative, since that would cause velocity to switch direction later in these calculations
        currentMPAcceleration = -lastMPVelocity / MPdt;
        currentMPVelocity = 0.0;
      }

      // Calculate the distance the climber should travel in the next interval
      currentMPDistance = currentMPDistance + (currentMPVelocity + lastMPVelocity) / 2.0 * MPdt;
      if (currentMPDistance > targetMPDistance || (approachingTarget && currentMPVelocity==0.0) ) {
        currentMPDistance = targetMPDistance;
      }

    // The profile has reached its target, so do not change the theoretical distance and zero the profile velocity and acceleration
    } else {
      currentMPDistance = targetMPDistance;
      lastMPVelocity = 0;
      currentMPVelocity = 0;
      currentMPAcceleration = 0;
    }
  }

  /**
   * Makes the climber follow the motion profile. This method should be called exactly once per scheduler cycle.
   * NOTE: Only follow the motion if climberPosControl is true. Otherwise, it is in manual control mode, so do nothing (return 0.0).
   * @return output percent based on calculations, -1.0 to +1.0 (positive = up, negative = down)
   */
  public double trackProfilePeriodic() {
    if (!profileEnabled) return 0.0;

    if (currentMPVelocity > 0 || Math.abs(percentPowerFB) > 0.008) {
      updateClimberProfileLog(false);
    }

    error = getCurrentPosition() - climber.getClimberPosition();
    updateProfileCalcs();
    intError = intError + error * measuredDt;

    // If the climber is moving up, use the profile terms for moving up
    if (directionSign == 1) {
      // percentPowerFF = kFF + kSu * Math.signum(currentMPVelocity * directionSign) + kVu * currentMPVelocity * directionSign + kAu * currentMPAcceleration * directionSign;
      percentPowerFF = kFF + kSu * Math.signum((currentMPVelocity + lastMPVelocity) * directionSign) + Bdu * directionSign * (currentMPVelocity - Adu * lastMPVelocity);
      percentPowerFB = kPu * error + ((error - prevError) * kDu) + (kIu * intError);

    // If the climber is moving down, use the profile terms for moving down
    } else if (directionSign == -1) {
      // percentPowerFF = kFF + kSd * Math.signum(currentMPVelocity * directionSign) + kVd * currentMPVelocity * directionSign + kAd * currentMPAcceleration * directionSign;
      percentPowerFF = kFF + kSd * Math.signum((currentMPVelocity + lastMPVelocity) * directionSign) + Bdd * directionSign * (currentMPVelocity - Add * lastMPVelocity);
      percentPowerFB = kPd * error + ((error - prevError) * kDd) + (kId * intError);

    // If the climber target = the initial position (i.e. directionSign = 0), then just use feedback
    } else {
      percentPowerFB = kPu * error + ((error - prevError) * kDu) + (kIu * intError);
      percentPowerFF = kFF + kSu * Math.signum(percentPowerFB);
    }

    prevError = error;

    // Cap feedback power to prevent jerking the climber
    percentPowerFB = MathUtil.clamp(percentPowerFB, -0.4, 0.4);

    return percentPowerFF + percentPowerFB;
  }

  /**
   * Writes information about the ClimberProfileGenerator to the file log.
   * @param logWhenDisabled true = write when robot is enabled or disabled, false = only write when robot is enabled
  */
  public void updateClimberProfileLog(boolean logWhenDisabled) {
    if (logWhenDisabled || !DriverStation.isDisabled()) {
      long timeNow = RobotController.getFPGATime();
      dLogTargetPos.append(finalPosition, timeNow);
      dLogTime.append(getTimeSinceProfileStart(), timeNow);
      dLogDt.append(measuredDt, timeNow);
      dLogMPPos.append(getCurrentPosition(), timeNow);
      dLogActualPos.append(climber.getClimberPosition(), timeNow);
      dLogMPVel.append(currentMPVelocity * directionSign, timeNow);
      dLogActualVel.append(climber.getClimberVelocity(), timeNow);
      dLogMPAccel.append(currentMPAcceleration * directionSign, timeNow);
      dLogPctFF.append(percentPowerFF, timeNow);
      dLogPctFB.append(percentPowerFB, timeNow);
    }
  }

  /**
   * Gets the current position of the climber, according to the profile.
   * @return current position, in inches
   */
  public double getCurrentPosition() {
    return currentMPDistance * directionSign + initialPosition;
  }

  /**
   * Gets the target/final position of the climber, according to the profile.
   * @return target position, in inches
   */
  public double getFinalPosition() {
    return finalPosition;
  }

  /**
   * Gets the velocity of the climber, according to the profile.
   * @return target velocity, in inches/second
   */
  public double getCurrentVelocity() {
    return currentMPVelocity * directionSign;
  }

  /**
   * Gets the time that has elapsed since the profile started.
   * @return elapsed time, in seconds
   */
  public double getTimeSinceProfileStart() {
    return ((double) (lastTime - startTime)) / 1000.0;
  }
}
