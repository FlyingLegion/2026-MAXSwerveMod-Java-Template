// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class DriveSubsystem extends SubsystemBase {
  public RobotContainer localRobotContainer;
  // Create MAXSwerveModules
  
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  public final Field2d m_field_odometry = new Field2d(); //Field Widget
  public final Field2d m_field_estimator = new Field2d(); //Field Widget



  // The navX sensor
  private final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
  private RobotConfig robotAutoConfig;

  private PIDController rotationPID= new PIDController(0.01, 0, 0);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(navX.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });


  SwerveDrivePoseEstimator m_estimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(navX.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d(new Translation2d(0,0),new Rotation2d(0))
      );

  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(RobotContainer m_robotContainer) {
    localRobotContainer = m_robotContainer;
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    try{
      robotAutoConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      System.out.println("Config Error");
    }

    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelativeSpeed, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(5, 0.0, 0.0), // Translation PID constants 
        new PIDConstants(5, 0, 0.0) // Rotation PID constants
      ),
      robotAutoConfig, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );


    SmartDashboard.putData("FieldOdometry", m_field_odometry);
    SmartDashboard.putData("FieldEstimator", m_field_estimator);

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(-navX.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    m_estimator.update(
        Rotation2d.fromDegrees(-navX.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    // Moved addVisionMeasurement into CameraModule
    // if(localRobotContainer.m_cameraSubsystem.cameraRobotPose2d().getTranslation().getDistance(new Translation2d(0,0)) != 0){
    //   m_estimator.addVisionMeasurement(localRobotContainer.m_cameraSubsystem.cameraRobotPose2d(), Timer.getFPGATimestamp());
    // }
    m_field_odometry.setRobotPose(this.getPose());
    m_field_estimator.setRobotPose(m_estimator.getEstimatedPosition());

  }


  /**
   * Returns the currently-estimated pose of the robot.
   *
   */
  public Pose2d getPose() {
    // return m_odometry.getPoseMeters();
    return m_estimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-navX.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
    
    m_estimator.resetPosition(
        Rotation2d.fromDegrees(-navX.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition(), 
          },
            pose);
    System.out.println("Reset Odometry: " + pose.toString());
  }
  
  public Command resetOdometryCommand(Pose2d pose) {
    return this.runOnce(() -> resetOdometry(pose));
  }

   public ChassisSpeeds getRobotRelativeSpeed() {
    var frontLeftState = m_frontLeft.getState();
    var frontRightState = m_frontRight.getState();
    var backLeftState = m_rearLeft.getState();
    var backRightState = m_rearRight.getState();
      // Convert to chassis speeds
      return DriveConstants.kDriveKinematics.toChassisSpeeds(
        frontLeftState, frontRightState, backLeftState, backRightState);
  }


  
  


  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-navX.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

   public void driveRobotRelativeSpeed(ChassisSpeeds speeds){
    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;
    double rot = speeds.omegaRadiansPerSecond;
    drive (vx, vy, rot, false);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public Command setXCommand() {
    return this.run(() -> setX());
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  } 
  // This will set the robot's position relative to the DriveTrain and will ideally communicate with the PoseEstimator located in CameraModule

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    System.out.println("Reset NAVX Heading");
    navX.reset();
  }
  
  public Command zeroHeadingCommand(){
    return this.runOnce(() -> zeroHeading());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-navX.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navX.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }




  

  //Alignment code for the robot
  public Translation2d radialOffset(double radius, double theta){
    //coordinates in relation to the goal
    Translation2d currentPolar = localRobotContainer.m_cameraSubsystem.getRelativePolar();
    Translation2d currentCartes = localRobotContainer.polarToCartesian(currentPolar);
    //target coordinates, also in relation to goal because current is
    Translation2d targetPolar = new Translation2d(radius, theta);
    Translation2d targetCartes = localRobotContainer.polarToCartesian(targetPolar);
    //pid calculationers
    double xmove = rotationPID.calculate(currentCartes.getX(), targetCartes.getX());
    double ymove = rotationPID.calculate(currentCartes.getY(), targetCartes.getY());
    Translation2d movement = new Translation2d(xmove,ymove);
    return movement;
  }

  public double getAngularSpeedToNearestGoal() {
    // input = -error - setpoint/target
    // if absolute value of error is more than 180, set the error to 360 * the sign of the error
    double curr = (m_estimator.getEstimatedPosition().getRotation().getDegrees() + 360) % 360; 
    double targ = 90;
    // Math.atan2(getPose().getY() - getNearestGoalCoords().getY(), getPose().getX() - getNearestGoalCoords().getX())

    
    // targ *= Constants.radiansToDegrees;
    
    return  rotationPID.calculate(curr, targ);
  }

  
    public Translation2d getNearestGoalCoords() {
        //Uses m_estimator pose estimator from drive subsystem
        Translation2d diffToRedGoal = Constants.FieldConstants.redGoal.minus(getPose().getTranslation());
        Translation2d diffToBlueGoal = Constants.FieldConstants.blueGoal.minus(getPose().getTranslation());
        //Uses distance formula to get vector length to find which is closest
        double blueMagnitude = localRobotContainer.translationMagnitude(diffToBlueGoal);
        double redMagnitude = localRobotContainer.translationMagnitude(diffToRedGoal);
        if(blueMagnitude < redMagnitude) {
            return Constants.FieldConstants.blueGoal;
        } else {
            return Constants.FieldConstants.redGoal;
        }
    }

}

