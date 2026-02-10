package frc.robot.subsystems;
import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;

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
import frc.robot.RobotContainer;


public class CameraModule extends SubsystemBase {
    //Basic Things
    public RobotContainer localRobotContainer;
    public PhotonCamera genericCamera;
    public boolean cameraHasTargets;
    public double heading;
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
    public double cameraZ;
    
    //AprilTag Map (VERY IMPORTANT)
    public static final AprilTagFieldLayout fieldLayout26 = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); //loads the apriltag layout

    public CameraModule(String cameraName, String cameraRemoteHost, RobotContainer m_robotContainer){
        localRobotContainer = m_robotContainer;
        PortForwarder.add(5800, cameraRemoteHost, 5800);

        genericCamera = new PhotonCamera(cameraName);
        cameraHasTargets = false;
        heading = 0;
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
        cameraZ = 0;

        System.out.println("Camera Module Initialized (" + cameraName + "" + cameraRemoteHost + "");
    }

    @Override
    public void periodic(){
        SmartDashboard.putString("Camera Modules", "Running");
        
        heading = localRobotContainer.m_robotDrive.getHeading();
        Rotation2d gyroAngle = new Rotation2d(Constants.degreesToRadians * heading);
            
        List<PhotonPipelineResult> results = genericCamera.getAllUnreadResults();
        if(!results.isEmpty()){
            PhotonPipelineResult result = results.get(results.size() - 1);
            cameraHasTargets = result.hasTargets();
            if(cameraHasTargets){
                robotPose = photonPoseEstimator.estimateCoprocMultiTagPose(genericCamera.getLatestResult());
                if(!robotPose.isEmpty()) {
                    cameraX = robotPose.get().estimatedPose.getX();
                    cameraY = robotPose.get().estimatedPose.getY();
                    cameraZ = robotPose.get().estimatedPose.getZ();
                    // System.out.println("("+cameraX+","+cameraY+","+cameraZ+")");
                    // This updates the robot's position relative to the Camera
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

    public void switchPipelineIndex(){
        if (pipelineIndex == 0) {
            genericCamera.setPipelineIndex(1);
        } else if (pipelineIndex == 1) {
            genericCamera.setPipelineIndex(0);
        }
    }

}


