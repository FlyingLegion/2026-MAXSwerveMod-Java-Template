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
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
    public boolean canGetPos;
    public int pipelineIndex;

    //Target Information
    public int targetID;
    public double targetYaw;
    public double targetPitch;

    //position things
    public PhotonPoseEstimator photonPoseEstimator;
    public Optional<EstimatedRobotPose> robotPose;

    //field relative XYZ positions
    public double cameraX;
    public double cameraY;
    public Translation2d cameraPos;
    public Rotation3d cameraRot;
    public double cameraHeading;

    //AprilTag Map (VERY IMPORTANT)
    public static final AprilTagFieldLayout fieldLayout26 = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); //loads the apriltag layout
    public final Field2d m_camField = new Field2d(); 
    
    public CameraModule(String cameraName, String cameraRemoteHost, Transform3d robotToCam, RobotContainer m_robotContainer){
        localRobotContainer = m_robotContainer;
        
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

        // robotToCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)); //overrode to 0 for debug
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout26, robotToCam);
        cameraX = 0;
        cameraY = 0;
        cameraPos = new Translation2d(0, 0);
        cameraRot = new Rotation3d(0, 0, 0);

        System.out.println("Camera Module Initialized (" + cameraName + "" + cameraRemoteHost + "");
    }

    @Override
    public void periodic(){
        SmartDashboard.putString("Camera Modules", "Running");
        SmartDashboard.putData("Field" + genericCamera.getName(), m_camField);



        List<PhotonPipelineResult> results = genericCamera.getAllUnreadResults();
        if(!results.isEmpty()){
            PhotonPipelineResult result = results.get(results.size() - 1);
            cameraHasTargets = result.hasTargets();
            if(cameraHasTargets){
                robotPose = photonPoseEstimator.estimateCoprocMultiTagPose(result);

                if(!robotPose.isEmpty()) {
                    canGetPos = true;
                    cameraX = robotPose.get().estimatedPose.getX();
                    cameraY = robotPose.get().estimatedPose.getY();
                    cameraRot = robotPose.get().estimatedPose.getRotation();
                    cameraHeading = cameraRot.getZ();
                    cameraPos = new Translation2d(cameraX, cameraY);//.minus(getCenterOffset(cameraRot));
                    //SmartDashboard.putString("robotPose3d " + genericCamera.getName(), robotPose.get().estimatedPose.toString());
                    //SmartDashboard.putString("robotPose2d " + genericCamera.getName(), robotPose.get().estimatedPose.toPose2d().toString());
                    //SmartDashboard.putString("robotRot " + genericCamera.getName(), robotPose.get().estimatedPose.getRotation().toString());
                    //SmartDashboard.putNumber("robotHeadingRad " + genericCamera.getName(), cameraHeading);
                    //SmartDashboard.putNumber("robotHeadingDeg " + genericCamera.getName(), cameraHeading*Constants.radiansToDegrees);
                    //m_camField.setRobotPose(robotPose.get().estimatedPose.toPose2d());
                    m_camField.setRobotPose(new Pose2d(cameraX, cameraY, new Rotation2d(cameraHeading)));

                } else {
                    canGetPos = false;
                    cameraX = 0;
                    cameraY = 0;
                    cameraRot = new Rotation3d(0,0,0);
                    cameraPos = new Translation2d(0,0);
                    m_camField.setRobotPose(new Pose2d(0,0, new Rotation2d(0.5,0.5)));
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

    public double getCameraHeading() {
        return cameraHeading;
    }

    public Translation2d getCameraPos() {
        return cameraPos;
    }
    public Translation2d getCenterOffset(Rotation3d rot){
        double theta = rot.getY();
        Translation2d cameraOffset = localRobotContainer.polarToCartesian(new Translation2d(Constants.CameraConstants.cornerRadius, theta));
        return cameraOffset;
    } 

    public void switchPipelineIndex(){
        if (pipelineIndex == 0) {
            genericCamera.setPipelineIndex(1);
        } else if (pipelineIndex == 1) {
            genericCamera.setPipelineIndex(0);
        }
    }

}


