package frc.robot.subsystems;
import java.util.*;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class CameraSubsystem extends SubsystemBase {  
    private RobotContainer localRobotContainer;

    public final CameraModule OrangeCamera;
    public final CameraModule WhiteCamera;
    public final CameraModule YellowCamera;
    public final CameraModule BlackCamera;
    // public final CameraModule ArduCam;
    //Arraylist of raspberry pi cameras
    public ArrayList<CameraModule> rpiCams = new ArrayList<CameraModule>();

    //Average Robot Position
    public double robotX;
    public double robotY;
    public double robotPos;
    public double distanceToGoal;
    public Translation2d diffToRedGoal;
    public Translation2d diffToBlueGoal;


    //GLASS VARIABLES
    public final Field2d m_field = new Field2d(); //Field Widget
    private static ShuffleboardTab cameraTab;
    
    
    public CameraSubsystem(RobotContainer m_robotContainer) {
        localRobotContainer = m_robotContainer;
        System.out.println("Camera Subsystem Initialized");

        // ArduCam = new CameraModule(
        //     "Orange_Pi_Cam",
        //     "photonvision",
        //     Constants.CameraConstants.orangePICameraOffset,
        //     m_robotContainer);

        OrangeCamera = new CameraModule(
            "Orange", 
            "photonvisionow",
            Constants.CameraConstants.orangeCameraOffset,
            m_robotContainer);
    
        WhiteCamera = new CameraModule(
            "White", 
            "photonvisionow",
            Constants.CameraConstants.whiteCameraOffset,
            m_robotContainer);

        YellowCamera = new CameraModule(
            "Yellow", 
            "photonvisionyb",
            Constants.CameraConstants.yellowCameraOffset,
            m_robotContainer);

        BlackCamera = new CameraModule(
            "Black", 
            "photonvisionyb",
            Constants.CameraConstants.blackCameraOffset,
            m_robotContainer);

        rpiCams.add(OrangeCamera);
        rpiCams.add(WhiteCamera);
        rpiCams.add(YellowCamera);
        rpiCams.add(BlackCamera);

        
        SmartDashboard.putData("FieldCameraAveraged", m_field);
    }

    @Override
    public void periodic() {
        m_field.setRobotPose(cameraRobotPose2d()); // rotation is read in radians
    }

    public void switchPipelineIndex(){
        // ArduCam.switchPipelineIndex();
    }

    public void loadShuffleboardTabs() {
        try {
            cameraTab = Shuffleboard.getTab("Camera");
            //test = cameraTab.add("testing", "").getEntry();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Shuffleboard Loading Error Caught");
        }
    }

    public void sendShuffleboardInfo() {
        try {
            //test.setString("not a variable yet ;()");
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Shuffleboard Variable Error");
        }
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
        if(count > 0) {
            return pos/count;
        } else {
            System.out.println("ERROR!!! COUNT IS 0 (getRobotX())");
            return -1;
        }
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
    

        if(count > 0) {
            return pos/count;
        } else {
            System.out.println("ERROR!!! COUNT IS 0 (getRobotY())");
            return -1;
        }
    }

    public double getRobotTheta() {
        double x = 0;
        double y = 0;
        int count = 0;

        for(var i = 0; i < rpiCams.size(); i++) {
            if(rpiCams.get(i).checkCanGetPos() == true) {
                x+=Math.cos(rpiCams.get(i).getCameraHeading());
                y+=Math.sin(rpiCams.get(i).getCameraHeading());
                count++;
            }
        }
        if(count > 0) {
            return Math.atan2(y/count,x/count);
        } else {
            System.out.println("ERROR!!! COUNT IS 0 (getRobotTheta())");
            return -1;
        }
    }
        
    public Translation2d getRobotPos() {
        Translation2d pos = new Translation2d(0,0);
        int count = 0;
        for(var i = 0; i < rpiCams.size(); i++) {
            if(rpiCams.get(i).checkCanGetPos() == true) {
                pos = pos.plus(rpiCams.get(i).getCameraPos());
                count++;
                // System.out.println("("+rpiCams.get(i).getCameraPos().getX()+","+rpiCams.get(i).getCameraPos().getY()+") (CAMERA) vs (FXN) ("+pos.getX()+","+pos.getY()+")"+"("+count+")");
            }
        }
        if(count > 0) {
            return pos.div(count);
        } else {
            //System.out.println("ERROR!!! COUNT IS 0 (getRobotPos())");
            return new Translation2d(0,0);
            
        }
    }

    public Object getRobotPos2() {
        try{
            Translation2d pos = new Translation2d(0,0);
            int count = 0;
            for(var i = 0; i < rpiCams.size(); i++) {
                if(rpiCams.get(i).checkCanGetPos() == true) {
                    pos = pos.plus(rpiCams.get(i).getCameraPos());
                    count++;
                    // System.out.println("("+rpiCams.get(i).getCameraPos().getX()+","+rpiCams.get(i).getCameraPos().getY()+") (CAMERA) vs (FXN) ("+pos.getX()+","+pos.getY()+")"+"("+count+")");
                }
            }
            if(count > 0) {
                return rpiCams.get(0)
                .genericCamera.getAllUnreadResults()
                .get(rpiCams.get(0).genericCamera.getAllUnreadResults().size() - 1)
                .hasTargets();
            } else {
                return rpiCams.get(0)
                .genericCamera.getAllUnreadResults()
                .get(rpiCams.get(0).genericCamera.getAllUnreadResults().size() - 1)
                .hasTargets();
            }
        } catch(Exception ex) { 
            System.out.println(ex);
            return "no";
        }
    }

    public Pose2d cameraRobotPose2d() {
        return new Pose2d(getRobotPos(), new Rotation2d(getRobotTheta()));
    }

    public void odometryTest() {
        localRobotContainer.m_robotDrive.resetOdometry(cameraRobotPose2d());
    }

    public Command cameraOdoCmd() {
        return this.runOnce(() -> odometryTest());
    }

    public Translation2d getRelativePolar() {
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
        return polar;
    }

}
