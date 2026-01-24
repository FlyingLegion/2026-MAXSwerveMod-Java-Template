package frc.robot.subsystems;
import java.util.List;

//Make a function that you pass it the camera to use and it gets all the values set - Do at some point because it will bevery good

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;
import frc.robot.Constants.CameraConstants;


public class CameraModule extends SubsystemBase {
    public RobotContainer localRobotContainer;
    public PhotonCamera genericCamera;
    public double heading;
    public int cameraFidID;
    public double cameraX;
    public double cameraY;
    public double cameraYaw;
    public double cameraPitch;
    public double targetDistanceMeters;
    public boolean cameraHasTargets;
    public Pose2d fieldToTarget2D;
    public Pose3d fieldToTarget3D;
    public Translation2d CameraToTargetTranslation;
    public Transform2d CameraToTarget;
    public int pipelineIndex;
    

    public static final AprilTagFieldLayout fieldLayout25 = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded); //loads the apriltag layout

    public CameraModule(String cameraName, String cameraRemoteHost, RobotContainer m_robotContainer){
        localRobotContainer = m_robotContainer;
        genericCamera = new PhotonCamera(cameraName); 
        PortForwarder.add(5800, cameraRemoteHost, 5800);
        
        heading = 0;
        cameraYaw = 0;
        cameraX = 0;
        cameraY = 0;
        cameraPitch = 0;
        cameraFidID = -1;
        cameraHasTargets = false;
        targetDistanceMeters = 0;
        pipelineIndex = genericCamera.getPipelineIndex();
        System.out.println("Camera Module Initialized (" + cameraName + "" + cameraRemoteHost + "");
        
    }

    @Override
    public void periodic(){
        SmartDashboard.putString("Camera Modules", "Running");

        heading = localRobotContainer.m_robotDrive.getHeading();
        Rotation2d gyroAngle = new Rotation2d(Constants.degreesToRadians * heading);

        if (pipelineIndex == 0) {
                    System.out.println("Using Ball Detection ");
            } else if (pipelineIndex == 1) {
                    System.out.println("Using AprilTag Detection ");
            } 
            
        List<PhotonPipelineResult> results = genericCamera.getAllUnreadResults();
        if(!results.isEmpty()){
            PhotonPipelineResult result = results.get(results.size() - 1);
            cameraHasTargets = result.hasTargets();
            if(cameraHasTargets){
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                cameraYaw = bestTarget.getYaw();
                cameraPitch = bestTarget.getPitch();
                
                //  AprilTag code
                /*
                cameraFidID = bestTarget.getFiducialId();
                fieldToTarget2D = fieldLayout25.getTagPose(cameraFidID).get().toPose2d();
                fieldToTarget3D = fieldLayout25.getTagPose(cameraFidID).get();
                
                targetDistanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
                    0.22225, //camera height; must correct
                    fieldToTarget3D.getZ(), //test target height
                    Constants.degreesToRadians * 15, //camera pitch - we are currently perpendicular to the ground; must change
                    Constants.degreesToRadians * cameraPitch); //Pitch of target

                CameraToTargetTranslation = PhotonUtils.estimateCameraToTargetTranslation(targetDistanceMeters, new Rotation2d(cameraYaw * Constants.degreesToRadians));
                CameraToTarget = PhotonUtils.estimateCameraToTarget(CameraToTargetTranslation, fieldToTarget2D, gyroAngle);
                cameraX = CameraToTarget.getX();
                cameraY = CameraToTarget.getY();
                */
            
            } else {
                cameraFidID = -1;
                cameraYaw = 0;
                cameraPitch = 0;
                targetDistanceMeters = 0;
            }
            // System.out.println("cameraYaw: " + cameraYaw);
            // System.out.println("cameraPitch: " + cameraPitch);
            // System.out.println("Camera Has Targets: " + cameraHasTargets);
        }
    }


    //Limelight Functions
    public boolean checkHasTargets(){
        return cameraHasTargets;
    }

    public double getYaw(){
        return cameraYaw;
    }
    public double getPitch(){
        return cameraPitch;
    }

    public double getXOffset(){
        return cameraX;
    }

    public double getYOffset(){
        return cameraY;
    }

    public int getTargetID() {
        return cameraFidID;
    }

    public double getTargetDist() {
        return targetDistanceMeters;
    }

    public int getPipelineIndex() {
        return genericCamera.getPipelineIndex();
    }

    public void switchPipelineIndex(){
        if (pipelineIndex == 0) {
            genericCamera.setPipelineIndex(1);
        } else if (pipelineIndex == 1) {
             genericCamera.setPipelineIndex(0);
        }
    }
}


