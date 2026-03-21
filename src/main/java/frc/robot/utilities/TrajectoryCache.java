// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.Constants.FieldConstants;


/**
 * Class that caches all trajectories that the robot could run.
 * Create one object instance of this class when the robot initializes to build the trajectories. 
 */
 public class TrajectoryCache {
    
   
    public enum TrajectoryName {
        // Add Choreo trajectories here.  Syntax:
        //   TrajectoryConstantName("Trajectory file name"),
        CollectThroughCenter("CollectThroughCenter"),
        CollectThroughSide("CollectThroughSide"),
        CollectEndToShootBump("CollectEndToShootBump"),
        CollectEndToShootTrench("CollectEndToShootTrench"),
        //StartBumpToCollectStart("StartBumpToCollectStart"),
        StartTrenchToCollectStart("StartTrenchToCollectStart"),
        ShootTrenchToOutpost("ShootTrenchToOutpost"),
        ShootTrenchToDepot("ShootTrenchToDepot"),
    
        Relative4mRotate180("Relative4mRotate180"),
        RelativeArcLeft("RelativeArcLeft"),
        RelativeStraight4m("RelativeStraight4m"),
        RelativeCirclePath("RelativeCirclePath"),
        RelativeRotate180("RelativeRotate180"),
        AbsoluteDiagonalTest("AbsoluteDiagonalTest"),

        DriveOverBump("DriveOverBump");

        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final String trajFileName;
        TrajectoryName(String trajFileName){ this.trajFileName = trajFileName; }
    }

    private static int trajectoryCount = TrajectoryName.values().length;

    // HashMap of Choreo Trajectories, generally  on the right side from driver perspective (2026)
    private HashMap<TrajectoryName, Trajectory<SwerveSample>> cache = new HashMap<TrajectoryName,Trajectory<SwerveSample>>(trajectoryCount);  
    

    /**
     * Load all trajectories when the robot starts up into a cache for trajectory-following commands.
     */
    public TrajectoryCache() {
        
        for (TrajectoryName tt : TrajectoryName.values() ) {
            cacheTrajectory(tt);
        }
    }

    /**
     * Loads the choreo trajectory into the trajectory cache that has
     * the same name as the trajectoryType enum's trajFileName String
     * If a trajectory with that String is unable to be loaded a sticky fault
     * will be recorded and no trajectory will be added to the trajectory cache
     * @param trajectoryLoad the trajectoryName enum that will be loaded into 
     * the trajectory cache with the enum as the key
     */
    public void cacheTrajectory(TrajectoryName trajectoryLoad){
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(trajectoryLoad.trajFileName);
        if(traj.isPresent()){
            cache.put(trajectoryLoad, traj.get());
        }
        else{
            RobotPreferences.recordStickyFaults("Trajectory-CacheTrajectory-" + trajectoryLoad.trajFileName +"-not-loaded");
        }
    }

    /**
     * Getter to get the choreo trajectory from the trajectory cache
     * @param key TrajectoryName key in the HashMap corresponding to the loaded trajectory
     * @return the Choreo trajectory in the trajectory cache.  Null if the requested trajectory
     * was not loaded successfully.
     */
    public Trajectory<SwerveSample> getTrajectory(TrajectoryName key){
        return cache.get(key);
    }

    /**
     * Getter to get the choreo trajectory from the trajectory cache 
     * @param leftside boolean; whether the needed trajectory is on the left side (return mirrored trajectory), or on the right side (return default trajectory) TODO handle side-specific trajectories (outpost/depot)
     * @param key TrajectoryName key in the HashMap corresponding to the loaded trajectory
     * @return the Choreo trajectory in the trajectory cache.  Null if the requested trajectory
     * was not loaded successfully.
     */
    public Trajectory<SwerveSample> getTrajectory(boolean leftSide, TrajectoryName key){
        return leftSide ? mirrorVertically(cache.get(key), "mirrored " + key) : cache.get(key);
    }

    /**
	 * Mirrors given trajectory vertically over the horizontal midline
	 * @param originalTraj Trajectory<SwerveSample> original trajectory
	 * @param newTrajName String name of the new trajectory
	 * @return Trajectory<SwerveSample> new mirrored trajectory
	 */
    public static Trajectory<SwerveSample> mirrorVertically(Trajectory<SwerveSample> originalTraj, String newTrajName) {
        var flippedStates = new ArrayList<SwerveSample>();
		for (var state : originalTraj.samples()) {
            flippedStates.add(mirrorSampleVertically(state));   // flip all of the data of each swerve sample
		}
		return new Trajectory<SwerveSample>(newTrajName, flippedStates, originalTraj.splits(), originalTraj.events());
    }

    /**
     * Mirrors swerve sample vertically over the horizontal midline (x stays the same, y is flipped)
     * @param sample SwerveSample given swerve sample
     * @return new mirrored swerve sample
     */
    public static SwerveSample mirrorSampleVertically(SwerveSample sample) {
        return new SwerveSample(
            sample.t, 
            sample.x,
            FieldConstants.width - sample.y,  // y-components get flipped; x components stay the same
            -sample.heading,
            sample.vx,
            -sample.vy,
            -sample.omega,
            sample.ax,
            -sample.ay,
            -sample.alpha,
            // FL, FR, BL, BR
            // Mirrored
            // FR, FL, BR, BL 
            new double[] {
                sample.moduleForcesX()[1], // FL and FR swerve modules flip
                sample.moduleForcesX()[0],
                sample.moduleForcesX()[3], // BL and BR swerve modules flip
                sample.moduleForcesX()[2]
            },
            // FL, FR, BL, BR
            // Mirrored
            // -FR, -FL, -BR, -BL
            new double[] {
                -sample.moduleForcesY()[1],
                -sample.moduleForcesY()[0],
                -sample.moduleForcesY()[3],
                -sample.moduleForcesY()[2]
        });
    }
}
