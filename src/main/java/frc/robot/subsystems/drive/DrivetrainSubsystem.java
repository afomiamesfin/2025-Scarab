// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.google.flatbuffers.Constants;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.team2052.lib.states.DrivetrainState;
import com.team2052.lib.swerve.SwerveModule;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.RobotState;
// import frc.robot.RobotState;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.BackLeftModule;
import frc.robot.Constants.DrivetrainConstants.BackRightModule;
import frc.robot.Constants.DrivetrainConstants.FrontLeftModule;
import frc.robot.Constants.DrivetrainConstants.FrontRightModule;
// import frc.robot.util.RobotStateEstimator;

public class DrivetrainSubsystem extends SubsystemBase {
    private final RobotState robotState = RobotState.getInstance();
    private final DriveController driveController = DriveController.getInstance();
    private final SwerveDriveKinematics kinematics = DrivetrainConstants.kinematics;

    final SwerveModule frontLeftModule;
    final SwerveModule frontRightModule;
    final SwerveModule backLeftModule;
    final SwerveModule backRightModule;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier fieldCentricSupplier;
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter rotationLimiter;

    private double lastAccelX = 0;
    private double lastAccelY = 0;

    private final AHRS navx;

    // old driving
    ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();

    // TODO: Drivetrain singleton??

    /** Creates a new SwerveDrivetrainSubsystem. */
    public DrivetrainSubsystem(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier fieldCentricSupplier) {

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.fieldCentricSupplier = fieldCentricSupplier;

        xLimiter = new SlewRateLimiter(2);
        yLimiter = new SlewRateLimiter(2);
        rotationLimiter = new SlewRateLimiter(5);
        frontLeftModule = new SwerveModule(
                "front left",
                FrontLeftModule.SwerveModuleConstants(),
                Ports.FL_ENCODER);
        frontRightModule = new SwerveModule( // crashes here
                "front right",
                FrontRightModule.SwerveModuleConstants(),
                Ports.FR_ENCODER);
        backLeftModule = new SwerveModule(
                "back left",
                BackLeftModule.SwerveModuleConstants(),
                Ports.BL_ENCODER);
        backRightModule = new SwerveModule(
                "back right",
                BackRightModule.SwerveModuleConstants(),
                Ports.BR_ENCODER);

        // navx = new AHRS(SPI.Port.kMXP, (byte) 200);
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

        zeroHeading();
    }

    // public void resetPose(Pose2d pose) {
    //     RobotStateEstimator.getInstance().resetPose(pose);
    // }

    @Override
    public void periodic() {
        // print encoder position
        // System.out.println("FRONT LEFT ENCODER POS: " + frontLeftModule.getState().angle.getDegrees());

        robotState.setDrivetrainState(
                new DrivetrainState(getRobotRelativeChassisSpeeds(), getModulePositions(), navx.getRotation2d()));

        robotState.updateCollisionDetected(getCollisionDetected());

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            getJoystickX() * DrivetrainSubsystem.getMaxVelocityMetersPerSecond(), 
            getJoystickY() * DrivetrainSubsystem.getMaxVelocityMetersPerSecond(), 
            getJoystickRotation() * DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond()
        );

        // The origin is always blue. When our alliance is red, X and Y need to be inverted
        var alliance = DriverStation.getAlliance();
        var invert = 1;
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            invert = -1;
        }

        if (getFieldCentric()) {
            chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond * invert, 
                chassisSpeeds.vyMetersPerSecond * invert, 
                chassisSpeeds.omegaRadiansPerSecond, 
                robotState.getRobotPose().getRotation()
            );
        }

        driveController.requestDriveAndTurn(DrivetrainConstants.DRIVER_PRIORITY, chassisSpeeds);

        // drive(driveController.getRequestedSpeeds());

        SmartDashboard.putBoolean("CollisionDetected", getCollisionDetected());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        // System.out.println("===== RUNNING WRONG DRIVE");
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
                chassisSpeeds,
                DrivetrainConstants.DRIVETRAIN_TO_ROBOT_CENTER_METERS);
        setModuleStates(swerveModuleStates);
    }

    // OLD DRIVE CHASSIS SPEEDS
    public void defaultDriveChassisSpeeds(ChassisSpeeds chassisSpeeds){
        // System.out.println("======= OLD DRIVE COMMAND");
        currentChassisSpeeds = chassisSpeeds;
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    // OLD DRIVE METHOD
    public void driveNCS(
        double normalizedXVelocity, 
        double normalizedYVelocity, 
        double normalizedRotationVelocity, 
        boolean fieldCentric
    ) {
        normalizedXVelocity = Math.copySign(
            Math.min(Math.abs(normalizedXVelocity), 1.0),
            normalizedXVelocity
        );
        normalizedYVelocity = Math.copySign(
            Math.min(Math.abs(normalizedYVelocity), 1.0),
            normalizedYVelocity
        );
        normalizedRotationVelocity = Math.copySign(
            Math.min(Math.abs(normalizedRotationVelocity), 1.0),
            normalizedRotationVelocity
        );

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            normalizedXVelocity * getMaxVelocityMetersPerSecond(), 
            normalizedYVelocity * getMaxVelocityMetersPerSecond(), 
            normalizedRotationVelocity * getMaxAngularVelocityRadiansPerSecond()
        );

        // The origin is always blue. When our alliance is red, X and Y need to be inverted
        var alliance = DriverStation.getAlliance();
        var invert = 1;
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            invert = -1;
        }

        // if (fieldCentric) {
            chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond * invert, 
                chassisSpeeds.vyMetersPerSecond * invert, 
                chassisSpeeds.omegaRadiansPerSecond, 
                Rotation2d.fromDegrees(-navx.getRotation2d().getDegrees())
            );
        System.out.println(navx.getRotation2d().getDegrees());
        // }

        defaultDriveChassisSpeeds(chassisSpeeds);
    }

    public void stop() {
        driveNCS(0, 0, 0, false);
    }

    public AHRS getNavx() {
        return navx;
    }

    public void zeroHeading() { // TODO: can't zero the heading
        navx.reset();
        // if (RobotState.getInstance().isRedAlliance()) {
        //     resetPose(new Pose2d(robotState.getRobotPose().getTranslation(), new Rotation2d(Math.toRadians(180))));
        // } else {
        //     resetPose(new Pose2d(robotState.getRobotPose().getTranslation(), new Rotation2d(0)));
        // }
    }

    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        // Check if the wheels don't have a drive velocity to maintain the current wheel
        // orientation.
        boolean hasVelocity = swerveModuleStates[0].speedMetersPerSecond != 0
                || swerveModuleStates[1].speedMetersPerSecond != 0
                || swerveModuleStates[2].speedMetersPerSecond != 0
                || swerveModuleStates[3].speedMetersPerSecond != 0;

        frontLeftModule.setState(
                swerveModuleStates[0].speedMetersPerSecond,
                hasVelocity ? swerveModuleStates[0].angle : frontLeftModule.getState().angle);
        frontRightModule.setState(
                swerveModuleStates[1].speedMetersPerSecond,
                hasVelocity ? swerveModuleStates[1].angle : frontRightModule.getState().angle);
        backLeftModule.setState(
                swerveModuleStates[2].speedMetersPerSecond,
                hasVelocity ? swerveModuleStates[2].angle : backLeftModule.getState().angle);
        backRightModule.setState(
                swerveModuleStates[3].speedMetersPerSecond,
                hasVelocity ? swerveModuleStates[3].angle : backRightModule.getState().angle);
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
    }

    public static double getMaxVelocityMetersPerSecond() {
        return SwerveModule.getMaxVelocityMetersPerSecond();
    }

    public static double getMaxAngularVelocityRadiansPerSecond() {
        /*
         * Find the theoretical maximum angular velocity of the robot in radians per
         * second
         * (a measure of how fast the robot can rotate in place).
         */

        return SwerveModule.getMaxVelocityMetersPerSecond() / Math.hypot(
                DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);
    }

    public boolean getCollisionDetected() {
        double currAccelX = navx.getWorldLinearAccelX();
        double currentJerkX = currAccelX - lastAccelX;
        lastAccelY = currAccelX;
        double currAccelY = navx.getWorldLinearAccelY();
        double currentJerkY = currAccelY - lastAccelY;
        lastAccelY = currAccelY;

        if ((Math.abs(currentJerkX) > DrivetrainConstants.COLLISION_THRESHOLD_DELTA_G)
                || (Math.abs(currentJerkY) > DrivetrainConstants.COLLISION_THRESHOLD_DELTA_G)) {
            return true;
        } else {
            return false;
        }
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        ChassisSpeeds robotRelative = getRobotRelativeChassisSpeeds();

        Rotation2d facing = RobotState.getInstance().getRobotPose().getRotation();
        Translation2d tx = new Translation2d(robotRelative.vxMetersPerSecond, robotRelative.vyMetersPerSecond)
                .rotateBy(facing);

        return new ChassisSpeeds(
                tx.getX(), tx.getY(),
                robotRelative.omegaRadiansPerSecond);
    }

    public void debug() {
        frontLeftModule.debug();
        frontRightModule.debug();
        backLeftModule.debug();
        backRightModule.debug();
    }

    private double getJoystickX() {
        return slewAxis(xLimiter, deadBand(-xSupplier.getAsDouble()));
    }

    private double getJoystickY() {
        return slewAxis(yLimiter, deadBand(-ySupplier.getAsDouble()));
    }

    private double getJoystickRotation() {
        return slewAxis(rotationLimiter, deadBand(rotationSupplier.getAsDouble()));
    }

    private boolean getFieldCentric() {
        return fieldCentricSupplier.getAsBoolean();
    }

    private double slewAxis(SlewRateLimiter limiter, double value) {
        return limiter.calculate(Math.copySign(Math.pow(value, 2), value));
    }

    private double deadBand(double value) {
        if (Math.abs(value) <= 0.075) {
            return 0.0;
        }
        // Limit the value to always be in the range of [-1.0, 1.0]
        return Math.copySign(Math.min(1.0, Math.abs(value)), value);
    }

    public enum DrivetrainAimStatus {
        AIMING,
        AIMED;
    }
}
