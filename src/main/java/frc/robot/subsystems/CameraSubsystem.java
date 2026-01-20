package frc.robot.subsystems;
import java.util.List;

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
import frc.robot.Constants.DriveConstants;

public class CameraSubsystem extends SubsystemBase {  

    private final CameraModule OrangeCamera;
    private final CameraModule WhiteCamera;
    private final CameraModule YellowCamera;
    private final CameraModule BlackCamera;

    private final CameraModule ArduCam;



    
    public CameraSubsystem(RobotContainer localRobotContainer) {
        System.out.println("Camera Subsystem Initialized");

        ArduCam = new CameraModule(
        "Ardu",
        "photonvision",
        localRobotContainer);

    
        OrangeCamera = new CameraModule(
            "Orange", 
            "photonvisionow",
            localRobotContainer);
    
        WhiteCamera = new CameraModule(
            "White", 
            "photonvisionow",
            localRobotContainer);

        YellowCamera = new CameraModule(
            "Yellow", 
            "photonvisionyb",
            localRobotContainer);

        BlackCamera = new CameraModule(
            "Black", 
            "photonvisionyb",
            localRobotContainer);
    }

    @Override
    public void periodic() {

        
        System.out.println("Yellow IDs: " + YellowCamera.getTargetID());
        System.out.println("Yellow Pitch: " + YellowCamera.getPitch());
        System.out.println("Target height: " + YellowCamera.fieldToTarget3D.getZ());
        System.out.println("Yellow Target Distance (in): " + YellowCamera.getTargetDist()*39.37);

        System.out.println("Black IDs: " + BlackCamera.getTargetID());
        System.out.println("Black Yaw: " + BlackCamera.getYaw());
        System.out.println("Black Target Distance: " + BlackCamera.getTargetDist());

        System.out.println("Orange IDs: " + OrangeCamera.getTargetID());
        System.out.println("Orange Yaw: " + OrangeCamera.getYaw());
        System.out.println("Orange Target Distance: " + OrangeCamera.getTargetDist());

        System.out.println("White IDs: " + WhiteCamera.getTargetID());
        System.out.println("White Yaw: " + WhiteCamera.getYaw());
        System.out.println("White Target Distance: " + WhiteCamera.getTargetDist());

    }
    
    
}
