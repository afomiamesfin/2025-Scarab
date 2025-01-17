package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DrivetrainConstants;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.subsystems.vision.VisionPoseAcceptor;
// import frc.robot.subsystems.vision.VisionUpdate;
import frc.robot.RobotState;

public class RobotStateEstimator {
    public static RobotStateEstimator INSTANCE;
    private RobotState robotState = RobotState.getInstance();

    private SwerveDrivePoseEstimator poseEstimator;
    // private VisionPoseAcceptor visionAcceptor;

    /**
     * RobotStateEstimator uses SwerveDrivePoseEstimator to estimate the pose of the
     * robot, field relative.
     */
    public static RobotStateEstimator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotStateEstimator();
        }

        return INSTANCE;
    }

    private RobotStateEstimator() {
        // visionAcceptor = new VisionPoseAcceptor();
    }

    /**
     * Update the SwerveDrivePoseEstimator with values from RobotState
     */
    public void update() {

        // if the swerve module positions are null, we haven't made the drivetrain yet
        if (robotState.getSwervePos() == null) {
            return;
        }

        if (poseEstimator == null) {
            poseEstimator = new SwerveDrivePoseEstimator(
                    DrivetrainConstants.kinematics,
                    robotState.getNavXRotation(),
                    robotState.getSwervePos(),
                    robotState.getInitialPose());
        }

        /*
         * Odometry updates
         */
        poseEstimator.update(
                robotState.getNavXRotation(),
                robotState.getSwervePos());

        if(poseEstimator.getEstimatedPosition() != null) {   
            System.out.println("======= ROTATION:" + poseEstimator.getEstimatedPosition().getRotation().getDegrees());
            robotState.updateRobotPose(poseEstimator.getEstimatedPosition());
        }
    }

    /**
     * Reset the position of SwerveDrivePoseEstimator and set the NavX Offset
     */
    public void resetPose(Pose2d pose) {
        if (poseEstimator != null) {
            poseEstimator.resetPosition(
                    robotState.getNavXRotation(),
                    robotState.getSwervePos(),
                    pose);
        }
    }
}
