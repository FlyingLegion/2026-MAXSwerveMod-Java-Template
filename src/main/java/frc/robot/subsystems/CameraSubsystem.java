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

    // private final CameraModule OrangeCamera;
    // private final CameraModule WhiteCamera;
    private final CameraModule YellowCamera;
    private final CameraModule BlackCamera;
    public final CameraModule ArduCam;

    
    public CameraSubsystem(RobotContainer localRobotContainer) {
        System.out.println("Camera Subsystem Initialized");

        ArduCam = new CameraModule(
            "Orange_Pi_Cam",
            "photonvision",
            localRobotContainer);

        // OrangeCamera = new CameraModule(
        //     "Orange", 
        //     "photonvisionow",
        //     localRobotContainer);
    
        // WhiteCamera = new CameraModule(
        //     "White", 
        //     "photonvisionow",
        //     localRobotContainer);

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
    }

    public void switchPipelineIndex(){
        ArduCam.switchPipelineIndex();
    }
    
    
}
