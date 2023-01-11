// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Robot;

import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {
  // Declare class-only constants and member variables before the Constructor below
  /////////////////// Subsystem Sensors and Actuators  //////////////////////
  private final WPI_TalonFX m_left_leader, m_right_leader, m_left_follower, m_right_follower;

  // The Navx Gyro
  private AHRS m_gyro;


  /////////////// Odometry and Kinematics  /////////////////////////
  private final DifferentialDriveOdometry m_odometry;
  private final Field2d m_field;

  ////////////////////// Simulator Specific Options /////////////////
  DifferentialDrivetrainSim m_drivetrain_sim;
  TalonFXSimCollection m_left_drive_sim, m_right_drive_sim;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    // Instantiate the member variables for the DT
    // First the NAVX
    m_gyro = new AHRS(SPI.Port.kMXP);

    // Falcons
    m_left_leader = new WPI_TalonFX(LEFT_TALON_LEADER);
    m_left_follower = new WPI_TalonFX(LEFT_TALON_FOLLOWER);
    m_right_leader = new WPI_TalonFX(RIGHT_TALON_LEADER);
    m_right_follower = new WPI_TalonFX(RIGHT_TALON_FOLLOWER);

    // Create blank configuration objects and "factory default" all the talons
    // This erases any settings, including those that were set in Phoenix tuner
    // before rebooting robot code
    TalonFXConfiguration _leftconfig = new TalonFXConfiguration();
    TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

    m_left_leader.configAllSettings(_leftconfig);
    m_left_follower.configAllSettings(_leftconfig);
    m_right_leader.configAllSettings(_rightConfig);
    m_right_follower.configAllSettings(_rightConfig);
  
    // Configure the talons with internal settings, THIS MUST BE DONE BEFORE SETTING
    // FOLLOWER TYPE, and STATUS FRAMES since these settings will destroy the 
    // settings that were set in code
    // Set the integrated encoder as the primary sensor
    _leftconfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    _rightConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    // Set output limits for forward and back
    _leftconfig.peakOutputForward = k_dt_peak_output_forward;
    _leftconfig.peakOutputReverse = k_dt_peak_output_reverse;
    _rightConfig.peakOutputForward = k_dt_peak_output_forward;
    _rightConfig.peakOutputForward = k_dt_peak_output_reverse;

    // Set deadband for motor controllers
    _leftconfig.neutralDeadband = k_dt_neutral_deadband;
    _rightConfig.neutralDeadband = k_dt_neutral_deadband;
    
    // Set Motor Configs with custom settings
    m_left_leader.configAllSettings(_leftconfig);
    m_right_leader.configAllSettings(_rightConfig);

    /* Continue to configure the rest of the talon configurations now */
    // Set Drivetrain inverts
    m_left_leader.setInverted(TalonFXInvertType.CounterClockwise); // invert = false
    m_left_follower.setInverted(InvertType.FollowMaster);
    m_right_leader.setInverted(TalonFXInvertType.Clockwise); // invert = true
    m_right_follower.setInverted(InvertType.FollowMaster);

    /* Set status frame periods */
		// Leader Talons need faster updates 
		m_right_leader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, 0);
		m_right_leader.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, 0);
		m_left_leader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);		//Used remotely by right Talon, speed up
		// Followers can slow down certain status messages to reduce the can bus usage, per CTRE:
		// "Motor controllers that are followers can set Status 1 and Status 2 to 255ms(max) using setStatusFramePeriod."
		m_right_follower.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
		m_right_follower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
		m_left_follower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
		m_left_follower.setStatusFramePeriod(StatusFrame.Status_1_General, 255);

    // Set the Encoders to Zero just to be sure
    m_right_leader.setSelectedSensorPosition(0);
    m_left_leader.setSelectedSensorPosition(0);

    /* Instantiate the simulator objects. This includes the drivetrain,
       and talking to the NAVX gyro */
    if( Robot.isSimulation() ){
      m_drivetrain_sim = new DifferentialDrivetrainSim(
        DCMotor.getFalcon500(2), // 2 falcons on each side of DT
        k_dt_gear_ratio, // Verify correct in Constants.java
        2.1, // Moment of Inertia of 2.1 kg m^2
        k_robot_mass, // Verify correct in Constants.java
        Units.inchesToMeters(k_dt_wheel_radius_inches),
        k_dt_track_width_meters, 
        null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) Uncomment this line to add measurement noise.
      );

      // Setup the simulation input classes
      m_left_drive_sim = m_left_leader.getSimCollection();
      m_right_drive_sim = m_right_leader.getSimCollection();
    }

    // Set the angle to 0 as the starting point
    m_gyro.reset();

    // Create the Odometry Object to estimate robot position on field.
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);

    // Create the field for Pose visualization and make it available on SmartDashboard Table
    m_field = new Field2d();
    SmartDashboard.putData(m_field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
