package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.team2052.lib.states.DrivetrainState;
import com.team2052.lib.status.AimStatus;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.vision.VisionUpdate;
// import frc.robot.util.AimingCalculator;
// import frc.robot.util.io.Dashboard;
// import frc.robot.util.io.Dashboard.ShooterIdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
    private static RobotState INSTANCE;

    private Pose2d initialPose;
    private Pose2d robotPose;
    private boolean visionEnabled;
    private Rotation2d navxRotation;
    private SwerveModulePosition[] swerveModulePositions;
    private ChassisSpeeds chassisSpeedsRobotRel;
    private boolean collisionDetected;
    private Field2d field2d;
    // private List<VisionUpdate> visionUpdates;
    private boolean noteHeld;
    private boolean noteStaged;
    private AimStatus hoodAimStatus;
    private AimStatus chassisAimStatus;
    private boolean isShooting;
    private boolean isIntaking;
    // private ShooterIdleMode currentShooterIdle;

    public static RobotState getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }

        return INSTANCE;
    }

    private RobotState() {
        initialPose = new Pose2d();
        robotPose = new Pose2d();
        visionEnabled = false;
        navxRotation = new Rotation2d(0);
        chassisSpeedsRobotRel = new ChassisSpeeds();
        collisionDetected = false;
        field2d = new Field2d();
        // visionUpdates = new ArrayList<VisionUpdate>();
        noteHeld = false;
        noteStaged = false;
        hoodAimStatus = AimStatus.INACTIVE;
        chassisAimStatus = AimStatus.INACTIVE;
        isShooting = false;
        isIntaking = false;
        // currentShooterIdle = ShooterConstants.DEFAULT_IDLE_MODE;
    }

    /* Setters */

    /**
     * Update current drivetrain state
     */
    public void setDrivetrainState(DrivetrainState drivetrainState) {
        this.chassisSpeedsRobotRel = drivetrainState.getChassisSpeeds();
        this.swerveModulePositions = drivetrainState.getModulePositions();
        this.navxRotation = drivetrainState.getRotation2d();
    }

    /**
     * Update the current robot pose
     */
    public void updateRobotPose(Pose2d robotPose) {
        // constrain robot pose rotation between 0-360 degrees
        robotPose = new Pose2d(robotPose.getTranslation(), Rotation2d.fromDegrees(MathUtil.inputModulus(robotPose.getRotation().getDegrees(), 0, 360)));
        this.robotPose = robotPose;
    }

    /**
     * Set collision detection status
     */
    public void updateCollisionDetected(boolean collisionDetected) {
        this.collisionDetected = collisionDetected;
    }

    /**
     * Adds an AprilTag vision update
     */
    // public void updateVisionUpdates(List<VisionUpdate> newVisionUpdates) {
    //     this.visionUpdates.clear();
    //     for (VisionUpdate visionUpdate : newVisionUpdates) {
    //         this.visionUpdates.add(visionUpdate);
    //     }
    // }

    /**
     * Update if the note is past the first loading break
     * 
     * @param noteHeld
     */
    public void updateNoteHeld(boolean noteHeld) {
        this.noteHeld = noteHeld;
    }

    /**
     * Update if the note is past the second staging beam break
     * 
     * @param noteStaged
     */
    public void updateNoteStaged(boolean noteStaged) {
        this.noteStaged = noteStaged;
    }

    /**
     * Update if the hood status (travel or on target)
     * 
     * @param hoodStatus
     */
    public void updateHoodStatus(AimStatus hoodAimStatus) {
        this.hoodAimStatus = hoodAimStatus;
    }

    public void updateChassisAimStatus(AimStatus chassisAimStatus) {
        this.chassisAimStatus = chassisAimStatus;
    }

    public void updateIsShooting(boolean isShooting) { 
        this.isShooting = isShooting;
    }

    public void updateIsIntaking(boolean isIntaking) {
        this.isIntaking = isIntaking;
    }

    /* Getters */

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public ChassisSpeeds getChassisRobotRel() {
        return chassisSpeedsRobotRel;
    }

    /**
     * Returns the norm of the measured velocity of the robot
     */
    public double getRobotVelocityNorm() {
        Twist2d twistVelocity = new Twist2d(
                chassisSpeedsRobotRel.vxMetersPerSecond,
                chassisSpeedsRobotRel.vyMetersPerSecond,
                chassisSpeedsRobotRel.omegaRadiansPerSecond);
        Translation2d translationVelocity = new Translation2d(twistVelocity.dx, twistVelocity.dy);
        translationVelocity = translationVelocity.rotateBy(robotPose.getRotation());

        Twist2d measuredVelocity = new Twist2d(
                translationVelocity.getX(),
                translationVelocity.getY(),
                twistVelocity.dtheta);

        if (measuredVelocity.dy == 0.0) {
            return Math.abs(measuredVelocity.dx);
        }

        return Math.hypot(measuredVelocity.dx, measuredVelocity.dy);
    }

    public ChassisSpeeds getChassisSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeedsRobotRel, robotPose.getRotation());
    }

    public Rotation2d getNavXRotation() {
        return navxRotation;
    }

    // public ShooterIdleMode getShooterIdleMode() {
    //     return currentShooterIdle;
    // }

    public boolean getCollisionDetected() {
        return collisionDetected;
    }

    public boolean getVisionEnabled() {
        return visionEnabled;
    }

    public Pose2d getInitialPose() {
        return initialPose;
    }

    // public List<VisionUpdate> getVisionUpdates() {
    //     return visionUpdates;
    // }

    /**
     * Returns true if the robot is on red alliance.
     *
     * @return True if the robot is on red alliance.
     */
    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return (alliance.get() == DriverStation.Alliance.Red) ? true : false;
        } else {
            return false;
        }
    }

    /**
     * Returns true if the robot is disabled.
     *
     * @return True if the robot is disabled.
     */
    public static boolean isDisabled() {
        return DriverStation.isDisabled();
    }

    /**
     * Returns true if the robot is enabled.
     *
     * @return True if the robot is enabled.
     */
    public static boolean isEnabled() {
        return DriverStation.isEnabled();
    }

    /**
     * Returns true if the robot is E-stopped.
     *
     * @return True if the robot is E-stopped.
     */
    public static boolean isEStopped() {
        return DriverStation.isEStopped();
    }

    /**
     * Returns true if the robot is in teleop mode.
     *
     * @return True if the robot is in teleop mode.
     */
    public static boolean isTeleop() {
        return DriverStation.isTeleop();
    }

    /**
     * Returns true if the robot is in autonomous mode.
     *
     * @return True if the robot is in autonomous mode.
     */
    public static boolean isAutonomous() {
        return DriverStation.isAutonomous();
    }

    /**
     * Returns true if the robot is in test mode.
     *
     * @return True if the robot is in test mode.
     */
    public static boolean isTest() {
        return DriverStation.isTest();
    }

    /*
     * Output data
     */

    // public void displayAllVisionPoses() {
    //     for (VisionUpdate vision : visionUpdates) {
    //         Pose2d pose = vision.estimatedPose.toPose2d();
    //         String name = "Camera " + vision.camera.getName();
    //         field2d.getObject(name).setPose(pose);
    //     }
    // }

    public void displayRobotPose() {
        field2d.setRobotPose(robotPose);
    }

    public boolean getNoteHeld() {
        return noteHeld;
        // return false;
    }

    public boolean getNoteStaged() {
        // return false;
        return noteStaged;
    }

    public SwerveModulePosition[] getSwervePos() {
        return swerveModulePositions;
    }

    public AimStatus getHoodAimStatus() {
        return hoodAimStatus;
    }

    public AimStatus getChassisAimStatus() {
        return chassisAimStatus;
    }

    public boolean getIsShooting() {
        return isShooting;
    }

    public boolean getIsIntaking() {
        return isIntaking;
    }



    public void output() {
        // displayRobotPose();
        // displayAllVisionPoses();

        // Dashboard.getInstance().putData("Field", field2d);

        Logger.recordOutput("Robot Pose", getRobotPose());
        // Logger.recordOutput("Rotation", navxRotation);
        Logger.recordOutput("Swerve Module Positions", getSwervePos());
        // Logger.recordOutput("Robot Chassis Speeds Robot Relative", chassisSpeedsRobotRel);
        // Logger.recordOutput("Collision Detected", collisionDetected);
        SmartDashboard.putBoolean("Note HELD", getNoteHeld());
        SmartDashboard.putBoolean("Note STAGED", getNoteStaged());
        
        // aiming calculator
        // double[] shotParams =  AimingCalculator.calculateSpeakerShot(robotPose, getChassisSpeedsFieldRelative(), isRedAlliance(), Dashboard.getInstance().getAimMode());
        // double goalAngle = shotParams[4];
        // double currentAngle = MathUtil.inputModulus(robotPose.getRotation().getDegrees(), 0, 360);
        // Logger.recordOutput("AIM GOAL ANGLE", goalAngle);
        // Logger.recordOutput("AIM CURRENT ANGLE", currentAngle);

        // for (VisionUpdate visionUpdate : visionUpdates) {
        //     Logger.recordOutput(visionUpdate.camera.getName() + " Pose", visionUpdate.estimatedPose);
        // }
    }
}
