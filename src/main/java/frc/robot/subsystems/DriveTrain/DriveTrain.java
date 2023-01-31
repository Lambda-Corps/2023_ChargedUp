// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.LEFT_TALON_FOLLOWER;
import static frc.robot.Constants.LEFT_TALON_LEADER;
import static frc.robot.Constants.RIGHT_TALON_FOLLOWER;
import static frc.robot.Constants.RIGHT_TALON_LEADER;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer.*;

public class DriveTrain extends SubsystemBase {
/**
	 * Using the configSelectedFeedbackCoefficient() function, scale units to 3600 per rotation.
	 * This is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
	 */
  public final static double kTurnTravelUnitsPerRotation = 3600;
    
  /**
   * Empirically measure what the difference between encoders per 360'
   * Drive the robot in clockwise rotations and measure the units per rotation.
   * Drive the robot in counter clockwise rotations and measure the units per rotation.
   * Take the average of the two.
   */
  public static final int kCountsPerRev = 2048;    // Encoder counts per revolution of the motor shaft.
  public static final double kSensorGearRatio = 10.71; // Gear ratio is the ratio between the *encoder* and the wheels. On the AndyMark
                            // drivetrain, encoders mount 1:1 with the gearbox shaft.
  public static final double kGearRatio = 10.71;   // Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead
                            // of on the gearbox.
  public static final double kWheelRadiusInches = 3.15;
  public static final int k100msPerSecond = 10;
  public final static int kEncoderUnitsPerRotation = 88554;
  public final static double kEncoderTicksPerDegree = kEncoderUnitsPerRotation / 360;
  public static final double kEncoderTicksPerInch = (kCountsPerRev * kGearRatio) / (2 * Math.PI * kWheelRadiusInches);
  private final int kTimeoutMs = 0;
  private final double kNeutralDeadband = 0.002;
  private final double kControllerDeadband = 0.1;
  private final double kTrackWidthMeters = .546;
  private final double kRobotMass = 55.3;

	private final double MAX_TELEOP_DRIVE_SPEED = 1.0;
	// private final double arbFF = 0.2;
	// TalonFX's for the drivetrain
	// Right side is inverted here to drive forward
	WPI_TalonFX m_left_leader, m_right_leader, m_left_follower, m_right_follower;
	// WPI_TalonFX m_left_leader, m_right_leader;//, m_left_follower, m_right_follower;

	// Variables to hold the invert types for the talons
	TalonFXInvertType m_left_invert, m_right_invert;
	
	// Gyro 
	public AHRS m_gyro;
	

	//testing
	public DigitalInput top_switch;

	public DigitalInput bottom_switch;

	// Auxilliary PID tracker
	// private boolean m_was_correcting = false;
	// private boolean m_is_correcting = false;

	public BooleanPublisher m_bottom_switch_publisher;
	public BooleanPublisher m_top_switch_publisher;

	///////////// Odometry Trackers //////////////
	// Odometry class for tracking robot pose
	private final DifferentialDriveOdometry m_odometry;
	private final Field2d m_2dField;
	
	///////////// Simulator Objects //////////////
	// These classes help us simulate our drivetrain
	public DifferentialDrivetrainSim m_drivetrainSimulator;
	/* Object for simulated inputs into Talon. */
	private TalonFXSimCollection m_leftDriveSim, m_rightDriveSim;

	// RateLimiters to try to keep from tipping over
	SlewRateLimiter m_forward_limiter, m_rotation_limiter;
	private double m_drive_absMax;

	//Networktables intializations
	public float gyrox, gyroy, gyroz ;
	
	public static DoublePublisher motorsValueLeftPub;
	public static DoublePublisher motorsValueRightPub;
	final BooleanPublisher motorState;

	public double rightMotorSpeed;
	public double leftMotorSpeed;

  	/** Creates a new DriveTrain. */
 	public DriveTrain() {
    m_gyro = new AHRS(SPI.Port.kMXP);
    m_left_leader = new WPI_TalonFX(LEFT_TALON_LEADER);
		m_left_follower = new  WPI_TalonFX(LEFT_TALON_FOLLOWER);
    m_right_leader = new WPI_TalonFX(RIGHT_TALON_LEADER);
		m_right_follower = new WPI_TalonFX(RIGHT_TALON_FOLLOWER);

    /** Invert Directions for Left and Right */
    m_left_invert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
    m_right_invert = TalonFXInvertType.Clockwise; //Same as invert = "true"

	top_switch = new DigitalInput(Constants.LIMIT_SWITCH_ONE);
    bottom_switch = new DigitalInput(Constants.LIMIT_SWITCH_TWO);

    /** Config Objects for motor controllers */
    TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

		// Set follower talons to default configs, and then follo–w their leaders
		m_left_follower.configAllSettings(_leftConfig);
		m_right_follower.configAllSettings(_rightConfig);
		m_left_follower.follow(m_left_leader);
		m_left_follower.setInverted(InvertType.FollowMaster);
		m_right_follower.follow(m_right_leader);
		m_right_follower.setInverted(InvertType.FollowMaster);

    		/* Set Neutral Mode */
		m_left_leader.setNeutralMode(NeutralMode.Brake);
		m_right_leader.setNeutralMode(NeutralMode.Brake);

		/* Configure output */
		m_left_leader.setInverted(m_left_invert);
		m_right_leader.setInverted(m_right_invert);
  
    	/* Configure the left Talon's selected sensor as integrated sensor */
		/* 
		 * Currently, in order to use a product-specific FeedbackDevice in configAll objects,
		 * you have to call toFeedbackType. This is a workaround until a product-specific
		 * FeedbackDevice is implemented for configSensorTerm
		 */
		_leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Feedback Source
		_rightConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

		// /* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		// _rightConfig.remoteFilter1.remoteSensorDeviceID = m_left_leader.getDeviceID(); //Device ID of Remote Source
		// _rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type
		
		// /* Setup difference signal to be used for turn when performing Drive Straight with encoders */
		// setRobotTurnConfigs(m_right_invert, _rightConfig);

		/* Config the neutral deadband. */
		_leftConfig.neutralDeadband = kNeutralDeadband;
		_rightConfig.neutralDeadband = kNeutralDeadband;

		/* max out the peak output (for all modes).  However you can
		 * limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftConfig.peakOutputForward = 1.0;
		_leftConfig.peakOutputReverse = -1.0;
		_rightConfig.peakOutputForward = +1.0;
		_rightConfig.peakOutputReverse = -1.0;
			
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */

		int closedLoopTimeMs = 1;
		_rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
   	_rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot0.allowableClosedloopError = 25;
		_rightConfig.slot1.allowableClosedloopError = 25;
		_leftConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		_leftConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		_leftConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
   	_leftConfig.slot3.closedLoopPeriod = closedLoopTimeMs;
		_leftConfig.slot0.allowableClosedloopError = 25;
		_leftConfig.slot1.allowableClosedloopError = 25;

  	/* APPLY the config settings */
		m_left_leader.configAllSettings(_leftConfig);
		m_right_leader.configAllSettings(_rightConfig);
		m_left_leader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 1.0), 0);
		m_right_leader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 1.0), 0);

		/* Set status frame periods */
		// Leader Talons need faster updates 
		m_right_leader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, kTimeoutMs);
		m_right_leader.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, kTimeoutMs);
		m_left_leader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);		//Used remotely by right Talon, speed up
		// Followers can slow down certain status messages to reduce the can bus usage, per CTRE:
		// "Motor controllers that are followers can set Status 1 and Status 2 to 255ms(max) using setStatusFramePeriod."
		m_right_follower.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
		m_right_follower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
		m_left_follower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
		m_left_follower.setStatusFramePeriod(StatusFrame.Status_1_General, 255);

		// setEncodersToZero();
		m_right_leader.setSelectedSensorPosition(0);
		m_left_leader.setSelectedSensorPosition(0);

		/// Odometry Tracker objects
		m_2dField = new Field2d();
		SmartDashboard.putData(m_2dField);
		m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);

		// Code for simulation within the DriveTrain Constructor
		if (Robot.isSimulation()) { // If our robot is simulated
			// This class simulates our drivetrain's motion around the field.
			/* Simulation model of the drivetrain */
			m_drivetrainSimulator = new DifferentialDrivetrainSim(
			  DCMotor.getFalcon500(2), // 2 Falcon 500s on each side of the drivetrain.
			  kGearRatio, // Standard AndyMark Gearing reduction.
			  2.1, // MOI of 2.1 kg m^2 (from CAD model).
			  kRobotMass, // Mass of the robot is 26.5 kg.
			  Units.inchesToMeters(kWheelRadiusInches), // Robot uses 3" radius (6" diameter) wheels.
			  kTrackWidthMeters, // Distance between wheels is _ meters.
	  
			  /*
			  * The standard deviations for measurement noise:
			  * x and y: 0.001 m
			  * heading: 0.001 rad
			  * l and r velocity: 0.1 m/s
			  * l and r position: 0.005 m
			  */
			  null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this
				  // line to add measurement noise.
			);
	  
			// Setup the Simulation input classes
			m_leftDriveSim = m_left_leader.getSimCollection();
			m_rightDriveSim = m_right_leader.getSimCollection();
		  } // end of constructor code for the simulation

		  // Setup the drive train limiting test variables
		  // Default the slew rate 3 meters per second
		  m_forward_limiter = new SlewRateLimiter(3);
		  m_rotation_limiter = new SlewRateLimiter(3);
		  m_drive_absMax = MAX_TELEOP_DRIVE_SPEED;
		  
		  m_gyro.reset();

		  //Networktables!
		  NetworkTable shuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard");

		  motorsValueLeftPub  = shuffleboard.getDoubleTopic("Left Motor Speed").publish();
		  motorsValueRightPub = shuffleboard.getDoubleTopic("Right Motor Speed").publish();
		  motorState = shuffleboard.getBooleanTopic("DriveTrain/MotorState").publish();

		  m_top_switch_publisher  = shuffleboard.getBooleanTopic("Top Limit Switch").publish();
		  m_bottom_switch_publisher = shuffleboard.getBooleanTopic("Bottom Limit Switch").publish();
		  
	}
	
	public void UpdateTopics(){
		motorsValueLeftPub.set(m_left_leader.get());
		motorsValueRightPub.set(m_right_leader.get());
		m_top_switch_publisher.set(top_switch.get());
		m_bottom_switch_publisher.set(bottom_switch.get());
	}

	@Override
	public void periodic() {
		m_odometry.update(m_gyro.getRotation2d(),
                      nativeUnitsToDistanceMeters(m_left_leader.getSelectedSensorPosition()),
                      nativeUnitsToDistanceMeters(m_right_leader.getSelectedSensorPosition()));
   		m_2dField.setRobotPose(m_odometry.getPoseMeters());
		   
		UpdateTopics();
		setSpeedVarible();
		get_top_limit_switch();
		get_bottom_limit_switch();
	}

	public Publisher get_top_limit_switch(){
		return m_top_switch_publisher;
	}
	public Publisher get_bottom_limit_switch(){
		return m_bottom_switch_publisher;
	}

	public double getgyrox(){
		return m_gyro.getPitch();
	}
	public double getgyroy(){
		return m_gyro.getRoll();
	}
	public double getgyroz(){
		return m_gyro.getAngle();
	}
	
	public void setSpeedVarible(){
		leftMotorSpeed  = m_left_leader.get();
		rightMotorSpeed = m_right_leader.get();
	}

	/** Deadband 5 percent, used on the gamepad */
	private double deadband(double value) {
		/* Upper deadband */
		if (value >= kControllerDeadband) {
      		return value;
    	}
		
		/* Lower deadband */
		if (value <= -kControllerDeadband) {
      		return value;
    	}

		/* Outside deadband */
		return 0;
  	}
  
  	/** Make sure the input to the set command is 1.0 >= x >= -1.0 **/
	private double clamp_drive(double value) {
		/* Upper deadband */
		if (value >= m_drive_absMax) {
     		return m_drive_absMax;
   		}

		/* Lower deadband */
		if (value <= -m_drive_absMax) {
      		return -m_drive_absMax;
    	}

    /* Outside deadband */
    return value;
  }

	public void teleop_drive(double forward, double turn){
		forward = deadband(forward);
		turn = deadband(turn);

		forward = clamp_drive(forward);
		turn = clamp_drive(turn);

		//forward = -m_forward_limiter.calculate(forward) * m_drive_absMax;
		if(forward != 0 || turn != 0) {
			forward = m_forward_limiter.calculate(forward) * m_drive_absMax;
			turn = m_rotation_limiter.calculate(turn) * m_drive_absMax;
		}

		var speeds = DifferentialDrive.curvatureDriveIK(forward, turn, true);

		if(Robot.isSimulation()){
			// Just set the motors
			m_right_leader.set(ControlMode.PercentOutput, speeds.right);
			m_left_leader.set(ControlMode.PercentOutput, speeds.left);
			return;
		}			
			// Just set the motors
	  m_right_leader.set(ControlMode.PercentOutput, speeds.right);
		m_left_leader.set(ControlMode.PercentOutput, speeds.left);
  }

	@Override
	public void simulationPeriodic() {
		/* Pass the robot battery voltage to the simulated Talon FXs */
		m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
		m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

		/*
			* CTRE simulation is low-level, so SimCollection inputs
			* and outputs are not affected by SetInverted(). Only
			* the regular user-level API calls are affected.
			*
			* WPILib expects +V to be forward.
			* Positive motor output lead voltage is ccw. We observe
			* on our physical robot that this is reverse for the
			* right motor, so negate it.
			*
			* We are hard-coding the negation of the values instead of
			* using getInverted() so we can catch a possible bug in the
			* robot code where the wrong value is passed to setInverted().
			*/
		m_drivetrainSimulator.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(),
								-m_rightDriveSim.getMotorOutputLeadVoltage());

		/*
			* Advance the model by 20 ms. Note that if you are running this
			* subsystem in a separate thread or have changed the nominal
			* timestep of TimedRobot, this value needs to match it.
			*/
		m_drivetrainSimulator.update(0.02);

		/*
			* Update all of our sensors.
			*
			* Since WPILib's simulation class is assuming +V is forward,
			* but -V is forward for the right motor, we need to negate the
			* position reported by the simulation class. Basically, we
			* negated the input, so we need to negate the output.
			*/
		m_leftDriveSim.setIntegratedSensorRawPosition(
						distanceToNativeUnits(
							m_drivetrainSimulator.getLeftPositionMeters()
						));
		m_leftDriveSim.setIntegratedSensorVelocity(
						velocityToNativeUnits(
							m_drivetrainSimulator.getLeftVelocityMetersPerSecond()
						));
		m_rightDriveSim.setIntegratedSensorRawPosition(
						distanceToNativeUnits(
							-m_drivetrainSimulator.getRightPositionMeters()
						));
		m_rightDriveSim.setIntegratedSensorVelocity(
						velocityToNativeUnits(
							-m_drivetrainSimulator.getRightVelocityMetersPerSecond()
						));

		int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
		SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
		angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
	}

	private int distanceToNativeUnits(double positionMeters){
		double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
		double motorRotations = wheelRotations * kSensorGearRatio;
		int sensorCounts = (int)(motorRotations * kCountsPerRev);
		return sensorCounts;
	}

	private int velocityToNativeUnits(double velocityMetersPerSecond){
		double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
		double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
		double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
		int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
		return sensorCountsPer100ms;
	}

	private double nativeUnitsToDistanceMeters(double sensorCounts){
		double motorRotations = (double)sensorCounts / kCountsPerRev;
		double wheelRotations = motorRotations / kSensorGearRatio;
		double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
	  return positionMeters;
	}

	public double getLeftEncoderValue(){
		return m_left_leader.getSelectedSensorPosition();
	}
	
	public double getRightEncoderValue(){
		return m_right_leader.getSelectedSensorPosition();
	}

	public double getRawAngle(){
		return m_gyro.getAngle();
	}

}
