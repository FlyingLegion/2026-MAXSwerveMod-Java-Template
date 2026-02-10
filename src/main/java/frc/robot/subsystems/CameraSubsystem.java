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
import java.util.ArrayList;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class CameraSubsystem extends SubsystemBase {  
    public RobotContainer localRobotContainer;

    private final CameraModule OrangeCamera;
    private final CameraModule WhiteCamera;
    private final CameraModule YellowCamera;
    private final CameraModule BlackCamera;
    public final CameraModule ArduCam;
    //Arraylist of raspberry pi cameras
    public ArrayList<CameraModule> rpiCams = new ArrayList<CameraModule>();

    //Average Robot Position
    public double robotX;
    public double robotY;
    public double robotPos;
    public double distanceToGoal;
    public Translation2d diffToRedGoal;
    public Translation2d diffToBlueGoal;
    
    public CameraSubsystem(RobotContainer m_robotContainer) {
        localRobotContainer = m_robotContainer;
        System.out.println("Camera Subsystem Initialized");

        ArduCam = new CameraModule(
            "Orange_Pi_Cam",
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

        rpiCams.add(OrangeCamera);
        rpiCams.add(WhiteCamera);
        rpiCams.add(YellowCamera);
        rpiCams.add(BlackCamera);
    }

    @Override
    public void periodic() {
        diffToRedGoal = Constants.FieldConstants.redGoal.minus(getRobotPos());
        diffToBlueGoal = Constants.FieldConstants.blueGoal.minus(getRobotPos());
        double blueMagnitude = localRobotContainer.translationMagnitude(diffToBlueGoal);
        double redMagnitude = localRobotContainer.translationMagnitude(diffToRedGoal);
        Translation2d polar = new Translation2d(0,0);
        if(blueMagnitude < redMagnitude) {
            polar = localRobotContainer.cartesianToPolar(diffToBlueGoal);
        } else {
            polar = localRobotContainer.cartesianToPolar(diffToRedGoal);
        }
        System.out.println("X,Y Averaged Positions: ("+getRobotX()+","+getRobotY()+")");
        System.out.println("X,Y Averaged Translation: ("+getRobotPos().getX()+","+getRobotPos().getY()+")");
        System.out.println("r,θ in relation to nearest goal: ("+polar.getX()+","+polar.getY()+")");

        //make translation to coordinate thing
    }

    public void switchPipelineIndex(){
        ArduCam.switchPipelineIndex();
    }
    
    public double getRobotX() {
        double pos = 0;
        double count = 0;
        for(var i = 0; i < rpiCams.size(); i++) {
            if(rpiCams.get(i).checkCanGetPos() == true) {
                pos += rpiCams.get(i).getCameraX();
                count++;
            }
        }
        return pos/count;
    }

    public double getRobotY() {
        double pos = 0;
        double count = 0;
        for(var i = 0; i < rpiCams.size(); i++) {
            if(rpiCams.get(i).checkCanGetPos() == true) {
                pos += rpiCams.get(i).getCameraY();
                count++;
            }
        }
        return pos/count;
    }
        
    public Translation2d getRobotPos() {
        Translation2d pos = new Translation2d(0,0);
        int count = 0;
        for(var i = 0; i < rpiCams.size(); i++) {
            if(rpiCams.get(i).checkCanGetPos() == true) {
                pos = pos.plus(rpiCams.get(i).getCameraPos());
                count++;
                //System.out.println("("+rpiCams.get(i).getCameraPos().getX()+","+rpiCams.get(i).getCameraPos().getY()+") (CAMERA) vs (FXN) ("+pos.getX()+","+pos.getY()+")"+"("+count+")");
            }
        }
        return pos.div(count);
    }
}
