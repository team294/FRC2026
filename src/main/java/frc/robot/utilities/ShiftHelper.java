package frc.robot.utilities;

import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;
import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShiftHelper {
    private AllianceSelection allianceSelection;
    private Alliance alliance;

    private Timer matchTimer;

    public ShiftHelper(AllianceSelection allianceSelection, Timer matchTimer) {
        this.allianceSelection = allianceSelection; 
        alliance = this.allianceSelection.getAlliance();

        this.matchTimer = matchTimer;
    }

    /**
     * 
     * @return If blue won auto or not depending on FMC game specific message  
     */
    public boolean blueWonAuto() {
        return DriverStation.getGameSpecificMessage().equals("B"); // TODO Test Blue won auto function 
    }

    /**
     * 
     * @param currentMatchTime match time counting up from 0 in seconds
     * @return time left in seconds in the current shift
     */
    public double timeLeftInShiftSeconds(double currentMatchTime) {
        if(currentMatchTime < 20) {
            return 20 - currentMatchTime;
        } else {
            double teleopMatchTime = currentMatchTime - 20; // Teleop Time
            if(teleopMatchTime < 10) {
                return 10 - teleopMatchTime; // Transition Shift
            } else {
                double afterTransitionTime = teleopMatchTime - 10;  // Active/Inactive Shifts
                if(afterTransitionTime < 100) {
                    return 25 - afterTransitionTime % 25;
                } else {
                    double endGameTime = afterTransitionTime - 100; // Endgame Shift
                    return Math.max(30 - endGameTime, 0);
                }
            }
        }
    }

    /**
     * 
     * @param currentMatchTime match time counting up from 0 in seconds
     * @return if the current shift is blue or not
     */
    public boolean isCurrentShiftBlue(double currentMatchTime) {
        if (blueWonAuto() ) {
            return !((currentMatchTime > 30 && currentMatchTime < 55) || (currentMatchTime > 80 && currentMatchTime < 105)); // 0:30 - 0:55 or 1:20 - 1:45
        } else {
            return !((currentMatchTime > 55 && currentMatchTime < 80) || (currentMatchTime > 105 && currentMatchTime < 130)); // 0:55 - 1:20 or 1:45 - 2:10
        }
    }

    /**
     * 
     * @param currentMatchTime
     * @return if the current shift is your alliance's
     */
    public boolean isCurrentShiftYours(double currentMatchTime) {
        if(currentMatchTime < 30 || currentMatchTime > 130) // If in auto+transition or endgame
            return true;
        if(alliance == Alliance.Blue) {
            return isCurrentShiftBlue(matchTimer.get());
        }
        if(alliance == Alliance.Red) {
            return !isCurrentShiftBlue(matchTimer.get());
        }
        return true;
    }

    public void periodic() {
        SmartDashboard.putNumber(MATCH_TIMER.value, Math.max(160-matchTimer.get(), 0));
        SmartDashboard.putNumber(SHIFT_TIMER.value, timeLeftInShiftSeconds(matchTimer.get()));
        SmartDashboard.putBoolean(IS_BLUE_SHIFT.value, isCurrentShiftBlue(matchTimer.get()));
        SmartDashboard.putBoolean(IS_YOUR_SHIFT.value, isCurrentShiftYours(matchTimer.get()));
    }
}