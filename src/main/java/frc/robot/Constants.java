// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team2052.lib.requests.DriveRequest;
import com.team2052.lib.requests.TurnRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import com.team2052.lib.swerve.SwerveConstants;
import com.team2052.lib.swerve.SwerveModule.SwerveModuleConstants;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    // Left-to-right distance between drivetrain wheels
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(24);
    // Front-to-back distance between drivetrain wheels
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(19);
    // translation from center of drivetrain to center of robot
    public static final Translation2d DRIVETRAIN_TO_ROBOT_CENTER_METERS = new Translation2d(
            Units.inchesToMeters(2.25),
            0);
    public static final double WHEEL_RADIUS = 0.0; // TODO: what's the actual radius

    public static final double MAX_AUTO_VELO = 4.0;

    /*
      * FL: 157.8 BL:3.5 BR:340.0 FR:188.9
      */

    public static final class FrontLeftModule {
        public static final double STEER_OFFSET_RADIANS = 0;// -Math.toRadians(155.8);

        public static SwerveModuleConstants SwerveModuleConstants() {
            return new SwerveModuleConstants(
                    Ports.FL_DRIVE,
                    Ports.FL_STEER,
                    STEER_OFFSET_RADIANS);
        }
    }

    public static final class FrontRightModule {
        public static final double STEER_OFFSET_RADIANS = 0;//-Math.toRadians(188.9);

        public static SwerveModuleConstants SwerveModuleConstants() {
            return new SwerveModuleConstants(
                    Ports.FR_DRIVE,
                    Ports.FR_STEER,
                    STEER_OFFSET_RADIANS);
        }
    }

    public static final class BackLeftModule {
        public static final double STEER_OFFSET_RADIANS = 0;//-Math.toRadians(3.5);

        public static SwerveModuleConstants SwerveModuleConstants() {
            return new SwerveModuleConstants(
                    Ports.BL_DRIVE,
                    Ports.BL_STEER,
                    STEER_OFFSET_RADIANS);
        }
    }

    public static final class BackRightModule {
        public static final double STEER_OFFSET_RADIANS = 0;//-Math.toRadians(340);

        public static SwerveModuleConstants SwerveModuleConstants() {
            return new SwerveModuleConstants(
                    Ports.BR_DRIVE,
                    Ports.BR_STEER,
                    STEER_OFFSET_RADIANS);
        }
    }

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2, DRIVETRAIN_WHEELBASE_METERS / 2),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2, -DRIVETRAIN_WHEELBASE_METERS / 2),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2, DRIVETRAIN_WHEELBASE_METERS / 2),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2, -DRIVETRAIN_WHEELBASE_METERS / 2));

    // public static final SwerveDriveKinematics kinematics = new
    // SwerveDriveKinematics(
    // // Front left
    // new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2, 0.298),
    // // Front right
    // new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2, -0.298),
    // // Back left
    // new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2, 0.178),
    // // Back right
    // new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2, -0.178)
    // );

    public static final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.1, 0.1, 0.1);

    public static final double COLLISION_THRESHOLD_DELTA_G = 1f;

    public static final double AIM_TOL_DEG = 3.0;

    /*
      * Lower means less priority for driving
      * Driver: 0
      * Auto: 1
      * Aim: 2
      * Snap: 3
      */

    public static final int DRIVER_PRIORITY = 0;
    public static final int AUTO_PRIORITY = 1;
    public static final int SNAP_PRIORITY = 2;
    public static final int AIM_PRIORITY = 3;

    public static final double SNAP_PID_kP = 2.5;
    public static final double SNAP_PID_kI = 0.0;
    public static final double SNAP_PID_kD = 0.1;
    public static final double SNAP_PID_kF = 0.0;

    public static final double ROTATIONAL_FEEDFORWARD_kS = 0;// 0.61;
    public static final double ROTATIONAL_FEEDFORWARD_kV = 0;// 12 * DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond();
    public static final double ROTATIONAL_FEEDFORAWRD_kA = 0.0;

    public static final DriveRequest NULL_DRIVE = new DriveRequest(Integer.MIN_VALUE, new Translation2d(0, 0));
    public static final TurnRequest NULL_TURN = new TurnRequest(Integer.MIN_VALUE, 0);
  }

  public static final class DashboardConstants {
    public static final String DRIVE_MODE_KEY = "Drive Mode";
    public static final String IDLE_MODE_KEY = "Current Idle Mode";
    // public static final String AUTO_COMPILED_KEY = "Auto Compiled";
}
}
