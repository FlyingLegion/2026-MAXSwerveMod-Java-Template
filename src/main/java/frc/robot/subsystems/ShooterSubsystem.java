package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    public RobotContainer localRobotContainer;

    private double currentShooterSpeed;
    private Slot0Configs pidConfigs;
    public double targetVelocity; 
    
    private TalonFX m_shooterFX;


    //shooter subsystem brance
    public ShooterSubsystem(RobotContainer localRobotContainer) {
        pidConfigs = new Slot0Configs();
        m_shooterFX = new TalonFX(ShooterConstants.kKrakenMotorID, "rio");
        targetVelocity = 0;

        pidConfigs.kS = 0; // Value to Overcome static Friction
        pidConfigs.kV = targetVelocity; // The Target Velocity converted to output voltage

        pidConfigs.kP = 0; // Multiplier/Proportion
        pidConfigs.kI = 0; // Integral
        pidConfigs.kD = 0; // Derivative
    }

    @Override
    public void periodic() {
        m_shooterFX.getConfigurator().apply(pidConfigs);


    }

}