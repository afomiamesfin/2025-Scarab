// package frc.robot.util;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import frc.robot.Constants.DrivetrainConstants;
// // import frc.robot.Constants.VisionConstants;
// // import frc.robot.subsystems.vision.VisionPoseAcceptor;
// // import frc.robot.subsystems.vision.VisionUpdate;
// import frc.robot.RobotState;

// public class RobotStateEstimator {
//     public static RobotStateEstimator INSTANCE;
//     private RobotState robotState = RobotState.getInstance();

//     private SwerveDrivePoseEstimator poseEstimator;
//     // private VisionPoseAcceptor visionAcceptor;

//     /**
//      * RobotStateEstimator uses SwerveDrivePoseEstimator to estimate the pose of the
//      * robot, field relative.
//      */
//     public static RobotStateEstimator getInstance() {
//         if (INSTANCE == null) {
//             INSTANCE = new RobotStateEstimator();
//         }

//         return INSTANCE;
//     }

//     private RobotStateEstimator() {
//         // visionAcceptor = new VisionPoseAcceptor();
//     }

//     /**
//      * Update the SwerveDrivePoseEstimator with values from RobotState
//      */
//     public void update() {

//         // if the swerve module positions are null, we haven't made the drivetrain yet
//         if (robotState.getSwervePos() == null) {
//             return;
//         }

//         if (poseEstimator == null) {
//             poseEstimator = new SwerveDrivePoseEstimator(
//                     DrivetrainConstants.kinematics,
//                     robotState.getNavXRotation(),
//                     robotState.getSwervePos(),
//                     robotState.getInitialPose(),
//                     DrivetrainConstants.ODOMETRY_STDDEV,
//                     VisionConstants.VISION_STDDEV);
//         }

//         /*
//          * Odometry updates
//          */
//         if (!robotState.getCollisionDetected()) {
//             poseEstimator.update(
//                     robotState.getNavXRotation(),
//                     robotState.getSwervePos());
//         }

//         /*
//          * Vision updates
//          */
//         for (VisionUpdate visionUpdate : robotState.getVisionUpdates()) {
//             if (visionAcceptor.shouldAccept(visionUpdate, robotState.getRobotVelocityNorm(), robotState.getRobotPose(), RobotState.isAutonomous())) {
//                 Pose2d visionPose = visionUpdate.estimatedPose.toPose2d();
//                 double highestAmbiguity = visionUpdate.highestAmbiguity;
//                 double robotVelo = robotState.getRobotVelocityNorm();

//                 poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
//                         calculateXYStdDevs(robotVelo),
//                         calculateXYStdDevs(robotVelo),
//                         calculateHeadingStdDevs(visionUpdate, highestAmbiguity)));

//                 poseEstimator.addVisionMeasurement(
//                         visionPose,
//                         visionUpdate.timestampSeconds);

//             }
//         }
//         if(poseEstimator.getEstimatedPosition() != null) {   
//             robotState.updateRobotPose( poseEstimator.getEstimatedPosition());
//         }
//     }

//     private static double calculateXYStdDevs(double robotVelo) {
//         double xyStdDevs = VisionConstants.XY_STDDEV;
//         // exponentially decrease vision trust when at a higher speed
//         double offset = (Math.pow(robotVelo, 2));

//         return Math.max(VisionConstants.XY_STDDEV, xyStdDevs * offset);
//     }

//     private static double calculateHeadingStdDevs(VisionUpdate update, double poseAmbiguity) {
//         double headingStdDev = VisionConstants.HEADING_STDDEV;

//         // if we see both speaker tags trust rotation from tags
//         if (update.foundTag1 && update.foundTag2 && poseAmbiguity < 0.05) {
//             headingStdDev = 0.7;
//         }

//         return headingStdDev;
//     }

//     /**
//      * Reset the position of SwerveDrivePoseEstimator and set the NavX Offset
//      */
//     public void resetPose(Pose2d pose) {
//         if (poseEstimator != null) {
//             poseEstimator.resetPosition(
//                     robotState.getNavXRotation(),
//                     robotState.getSwervePos(),
//                     pose);
//         }
//     }
// }
