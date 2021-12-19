// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class DriveConstants {
        public static final byte mFrontLeft = 5;
        public static final byte mFrontRight = 2;
        public static final byte mRearLeft = 12;
        public static final byte mRearRight = 3;
    
        public static final double kDriveGearing = 12;
        
        public static final double mWheelDiameter = 0.1016;

        public static final double TRACK_WIDTH = 0.6604;

        public static final boolean GYRO_REVERSE = false;
        
        public static final double kP = 1.69, kD = 0.0, kI = 0.0;

        public static final double kV = 4.87, kS = 0.238, kA = 0.665;
    }

    
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 7;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);
      }
}
