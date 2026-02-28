// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

//subsystems
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CameraModule;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem(this);
  public CameraSubsystem m_cameraSubsystem = new CameraSubsystem(this);
  public Robot m_robot = new Robot();
  private final SendableChooser<Command> autoChooser;
 
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  //CommandXboxController m_opController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  PIDController pidRotationController = new PIDController(0.004, 0, 0); //Glass program Overides Coded PIDs
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Choose Your Auto:", autoChooser);

     // NamedCommands.registerCommand("Scorer Score", m_coralScorer.scorerIntakeInCommand(0.15));

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
private void configureButtonBindings() {
    m_driverController.rightTrigger()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
            
    m_driverController.leftBumper()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(0.15, 0.0, 0.0 ,false),
            m_robotDrive));
    
    // m_driverController.rightBumper()
    //     .whileTrue(new RunCommand(
    //     () ->  m_robotDrive.drive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    //                               -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
    //                               pidRotationController.calculate(m_cameraSubsystem.ArduCam.targetYaw, 0), 
    //                               false),
    //          m_robotDrive));
    
    /* Manual reset for robot orientation */
    m_driverController.start()
        .onTrue(m_robotDrive.zeroHeadingCommand());
    
    m_driverController.a()
        .whileTrue(new RunCommand(
            () -> System.out.println(m_robotDrive.getPose().getRotation())));

    m_driverController.b()
        .whileTrue(new RunCommand(
         () -> m_robotDrive.rotateToHeading(Math.atan2(m_robotDrive.getNearestGoalCoords().getY() - m_robotDrive.getPose().getY(), 
                                                       m_robotDrive.getNearestGoalCoords().getX() - m_robotDrive.getPose().getX())
                                    * Constants.radiansToDegrees, 
                                     -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                     -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband))));

    m_driverController.y()
      .whileTrue(new RunCommand(() -> m_robotDrive.drive(
        m_robotDrive.radialOffset(m_robot.globalRadiusTarget, m_robot.globalThetaTarget).getY(),
        m_robotDrive.radialOffset(m_robot.globalRadiusTarget, m_robot.globalThetaTarget).getX(),
        0,
        true),m_robotDrive));

    m_driverController.x()
     .whileTrue(new RunCommand(() -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        m_robotDrive.getAngularSpeedToNearestGoal(), 
        false), 
      m_robotDrive));
    
    m_driverController.back()
      .onTrue(m_cameraSubsystem.cameraOdoCmd());

}

public Command getAutonomousCommand(int autoSelected) {
  var autoArraySize = 20;
  String autoArray[] = new String[autoArraySize];

  autoArray[0] = "Test Auto";

  return new PathPlannerAuto(autoArray[autoSelected]);
}
  
  public Translation2d polarToCartesian(Translation2d polar) {
    //(r,theta)
    double r = polar.getX();
    double theta = polar.getY();
    double x = r * Math.cos(theta);
    double y = r * Math.sin(theta);
    Translation2d cartesian = new Translation2d(x,y);
    return cartesian;
  }
  
  public Translation2d cartesianToPolar(Translation2d cartesian) {
    //(r,theta)
    double x = cartesian.getX();
    double y = cartesian.getY();
    double r = Math.sqrt(x * x + y * y);
    double theta = Math.atan(y/x);
    Translation2d polar = new Translation2d(r,theta);
    return polar;
  }

  
  public double translationMagnitude(Translation2d vector) {
    return Math.sqrt(Math.pow(vector.getX(),2) + Math.pow(vector.getY(),2));
  }
}
