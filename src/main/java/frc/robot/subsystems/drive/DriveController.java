package frc.robot.subsystems.drive;

import com.team2052.lib.requests.DriveRequest;
import com.team2052.lib.requests.TurnRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveController {

    private static final DriveRequest NULL_DRIVE = new DriveRequest(Integer.MIN_VALUE, new Translation2d(0, 0));
    private static final TurnRequest NULL_TURN = new TurnRequest(Integer.MIN_VALUE, 0);

    private DriveRequest currentDriveRequest;
    private TurnRequest currentTurnRequest;
    private int lastSelectedPriority;

    private ChassisSpeeds requestedSpeeds;

    public static DriveController INSTANCE;
    public static DriveController getInstance() {
        if(INSTANCE == null) {
            INSTANCE = new DriveController();
        }
        
        return INSTANCE;
    }

    private DriveController() {
        currentDriveRequest = NULL_DRIVE;
        currentTurnRequest = NULL_TURN;
    }

    public synchronized void requestDriveAndTurn(int priority, ChassisSpeeds speeds) {
        requestDrive(new DriveRequest(priority, new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)));
        requestTurn(new TurnRequest(priority, speeds.omegaRadiansPerSecond));
    }

    public synchronized void requestDrive(DriveRequest request) {
        if (request.priority >= currentDriveRequest.priority) {
            currentDriveRequest = request;
        }
    }

    public synchronized void requestTurn(TurnRequest request) {
        if (request.priority >= currentTurnRequest.priority) {
            currentTurnRequest = request;
        }
    }

    public synchronized void stop() {
        currentDriveRequest = NULL_DRIVE;
        currentTurnRequest = NULL_TURN;
    }

    public ChassisSpeeds getRequestedSpeeds() {
        requestedSpeeds = new ChassisSpeeds(
                currentDriveRequest.robotTranslation.getX(),
                currentDriveRequest.robotTranslation.getY(),
                currentTurnRequest.rotationRadPerSecond);
        lastSelectedPriority = Math.max(currentDriveRequest.priority, currentTurnRequest.priority);
        currentDriveRequest = NULL_DRIVE;
        currentTurnRequest = NULL_TURN;

        return requestedSpeeds;
    }

    public int getLastSelectedPriority() {
        return lastSelectedPriority;
    }
}
