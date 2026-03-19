package frc.robot.utilities;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShiftHelper {
    private final Alliance alliance;
    private final Timer teleopTimer;

    /**
     * Keeps track of shifts, publishing data to Elastic regarding whether or not shift is active.
     * @param alliance Current Alliance (Alliance.Blue, Alliance.Red)
     * @param teleopTimer Timer object tracking match time
     */
    public ShiftHelper(Alliance alliance, Timer teleopTimer) {
        this.alliance = alliance;
        this.teleopTimer = teleopTimer;
    }

    /**
     * Returns whether or not Blue Alliance scored more points in autonomous based on FMS message.
     * @return true = blue won, false = red won  
     */
    public boolean blueWonAuto() {
        return DriverStation.getGameSpecificMessage().equals("B"); // TODO Test Blue won auto function
    }

    /**
     * Returns time left in the current scoring shift, regardless of which Alliance's it is.
     * @return time left in the current shift, in seconds
     */
    public double timeLeftInShiftSeconds() {
        if (teleopTimer.get() <= 0.0) {return 0.0;}

        if (teleopTimer.get() <= 10.0) {return 10.0 - teleopTimer.get();}
        else if (teleopTimer.get() <= 35.0) {return 35.0 - teleopTimer.get();}
        else if (teleopTimer.get() <= 60.0) {return 60.0 - teleopTimer.get();}
        else if (teleopTimer.get() <= 85.0) {return 85.0 - teleopTimer.get();}
        else if (teleopTimer.get() <= 110.0) {return 110.0 - teleopTimer.get();}
        else {return 140.0 - teleopTimer.get();}
    }

    /**
     * Returns whether or not it is Blue Alliance's shift
     * @return true = Blue Alliance scoring shift, false = Red Alliance scoring shift. ignores transition and endgame
     */
    public boolean isCurrentShiftBlue() {
        boolean firstOrThirdShift = (teleopTimer.get() > 10.0 && teleopTimer.get() <= 35.0) || (teleopTimer.get() > 85.0);
        return blueWonAuto() ? !firstOrThirdShift : firstOrThirdShift;
    }

    /**
     * Returns whether or not your hub is active.
     * @return true = your scoring shift (transition, 2 alliance-specific, endgame), false = not your shift (other 2 alliance-specific)
     */
    public boolean isCurrentShiftYours() {
        if (teleopTimer.get() <= 0) {return false;}

        if (teleopTimer.get() <= 10 || teleopTimer.get() >= 130) {
            return true;
        } else if (alliance == Alliance.Blue) {
            return isCurrentShiftBlue();
        } else {
            return !isCurrentShiftBlue();
        }
    }

    public void periodic() {
        SmartDashboard.putNumber(TELEOP_TIMER.value, Math.max(140.0 - teleopTimer.get(), 0));
        SmartDashboard.putNumber(SHIFT_TIMER.value, timeLeftInShiftSeconds());
        SmartDashboard.putBoolean(IS_BLUE_SHIFT.value, isCurrentShiftBlue());
        SmartDashboard.putBoolean(IS_YOUR_SHIFT.value, isCurrentShiftYours());
    }
}