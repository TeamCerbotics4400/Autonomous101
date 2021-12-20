// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;

/** Add your docs here. */
public class trajectoryS {
    public static Trajectory generatedTrajectoryOne(){
        var trajectoryWaypoints = new ArrayList<Pose2d>();

        var start = new Pose2d(1, 1, new Rotation2d(0));
        trajectoryWaypoints.add(start);

        var firstWayPoint = new Pose2d(2, 2, new Rotation2d(Math.toRadians(0)));
        trajectoryWaypoints.add(firstWayPoint);
    
        var secondWayPoint = new Pose2d(3, 1, new Rotation2d(Math.toRadians(0)));
        trajectoryWaypoints.add(secondWayPoint);

        var thirdWayPoint = new Pose2d(4, 2, new Rotation2d(0));
        trajectoryWaypoints.add(thirdWayPoint);

        var fourthWayPoint = new Pose2d(1, 0, new Rotation2d(180));
        trajectoryWaypoints.add(fourthWayPoint);
        
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(AutoConstants.kDriveKinematics);

        config.setReversed(false);
        var trajectory = TrajectoryGenerator.generateTrajectory(trajectoryWaypoints, config);
        return trajectory;
    }
}
