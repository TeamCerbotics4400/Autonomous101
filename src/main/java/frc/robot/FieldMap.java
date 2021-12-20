// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

/** Add your docs here. */

public class FieldMap {

    // Field marks, for reference
    public static final double fieldWidth = Units.inchesToMeters(323);
    public static final double fieldLength = Units.inchesToMeters(629.25);
    // Distance to the center of the white tape line
    public static final double startLineX = Units.inchesToMeters(1);

    // Robot dimensions with bumpers
    public static final double robotWidth = Units.inchesToMeters(34.75);
    public static final double robotLength = Units.inchesToMeters(37.5);

    // half dimensions - for getting the center point of the robot
    public static final double rW2 = robotWidth/2.0;
    public static final double rL2 = robotLength/2.0;
    
    public static Pose2d[] startPosition = new Pose2d[5];
    
    public static ArrayList<Translation2d> wayPointsA = new ArrayList<Translation2d>();

    static {

        // start positions are in terms of the robot center (x,y)

        // lined up with target center
        // 40 inches closer to center of field
        startPosition[0] = new Pose2d(1, 1, new Rotation2d(0));
        startPosition[1] = new Pose2d(2, 2, new Rotation2d(0));
        startPosition[2] = new Pose2d(3, 1, new Rotation2d(0));
        startPosition[3] = new Pose2d(4, 2, new Rotation2d(0));
        startPosition[4] = new Pose2d(1, 0, new Rotation2d(180));
    };
}
