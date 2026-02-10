package frc.robot.subsystems;
import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;

import javax.net.ssl.CertPathTrustManagerParameters;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class CameraModule extends SubsystemBase {
    //Basic Things
    public PhotonCamera genericCamera;
    public boolean cameraHasTargets;
    public boolean canGetPos;
    public int pipelineIndex;

    //Target Information
    public int targetID;
    public double targetYaw;
    public double targetPitch;

    //position things
    public Transform3d robotToCam;
    public PhotonPoseEstimator photonPoseEstimator;
    public Optional<EstimatedRobotPose> robotPose;

    //field relative XYZ positions
    public double cameraX;
    public double cameraY;
    public Translation2d cameraPos;
    public Translation2d vectorOffset;
    
    //AprilTag Map (VERY IMPORTANT)
    public static final AprilTagFieldLayout fieldLayout26 = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); //loads the apriltag layout

    public CameraModule(String cameraName, String cameraRemoteHost, Translation2d vectorMult){
        PortForwarder.add(5800, cameraRemoteHost, 5800);

        genericCamera = new PhotonCamera(cameraName);
        cameraHasTargets = false;
        canGetPos = false;
        pipelineIndex = genericCamera.getPipelineIndex();
        //Pipeline Index 0 -> Ball Detection
        //Pipeline Index 1 -> AprilTag Detection

        targetID = 0;
        targetYaw = 0;
        targetPitch = 0;

        robotToCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout26, robotToCam);
        cameraX = 0;
        cameraY = 0;
        cameraPos = new Translation2d(0, 0);
        vectorOffset = vectorMult.times(Constants.CameraConstants.cornerRadius);

        System.out.println("Camera Module Initialized (" + cameraName + "" + cameraRemoteHost + "");
    }

    @Override
    public void periodic(){
        SmartDashboard.putString("Camera Modules", "Running");
            
        List<PhotonPipelineResult> results = genericCamera.getAllUnreadResults();
        if(!results.isEmpty()){
            PhotonPipelineResult result = results.get(results.size() - 1);
            cameraHasTargets = result.hasTargets();
            if(cameraHasTargets){
                robotPose = photonPoseEstimator.estimateCoprocMultiTagPose(genericCamera.getLatestResult());

                if(!robotPose.isEmpty()) {
                    canGetPos = true;
                    cameraX = robotPose.get().estimatedPose.getX() - vectorOffset.getX();
                    cameraY = robotPose.get().estimatedPose.getY() - vectorOffset.getY();
                    cameraPos = new Translation2d(cameraX, cameraY).plus(vectorOffset);
                    // System.out.println("("+cameraX+","+cameraY+","+cameraZ+")");
                    // This updates the robot's position relative to the Camera
                } else {
                    canGetPos = false;
                    cameraX = 0;
                    cameraY = 0;
                    cameraPos = new Translation2d(0,0);
                }
                
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                targetID = bestTarget.getFiducialId();
                targetYaw = bestTarget.getYaw();
                targetPitch = bestTarget.getPitch();
            
            } else {
                targetID = -1;
                targetYaw = 0;
                targetPitch = 0;
            }
        }
    }

    public Pose2d getPose2dValues(){
        return new Pose2d(cameraX, cameraY, new Rotation2d(0)); // To do: please supply the rotation and calculate pose2d globally instead of Peridic
    }

    public boolean checkHasTargets(){
        return cameraHasTargets;
    }

    public boolean checkCanGetPos() {
        return canGetPos;
    }

    public double getYaw(){
        return targetYaw;
    }
    public double getPitch(){
        return targetPitch;
    }

    public int getTargetID() {
        return targetID;
    }

    public int getPipelineIndex() {
        return genericCamera.getPipelineIndex();
    }

    public double getCameraX() {
        return cameraX;
    }
    public double getCameraY() {
        return cameraY;
    }
    public Translation2d getCameraPos() {
        return cameraPos;
    }

    public void switchPipelineIndex(){
        if (pipelineIndex == 0) {
            genericCamera.setPipelineIndex(1);
        } else if (pipelineIndex == 1) {
            genericCamera.setPipelineIndex(0);
        }
    }

}


