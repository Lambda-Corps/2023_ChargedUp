// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.LEFT_TALON_FOLLOWER;
import static frc.robot.Constants.LEFT_TALON_LEADER;
import static frc.robot.Constants.PID_PRIMARY;
import static frc.robot.Constants.RIGHT_TALON_FOLLOWER;
import static frc.robot.Constants.RIGHT_TALON_LEADER;
import static frc.robot.Constants.kSlot_DriveMM;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

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
  public static final double kGearRatio = 4.17;   // Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead
                            // of on the gearbox.
  public static final double kWheelRadiusInches = 2.9;
  public static final int k100msPerSecond = 10;
  public static final double kEncoderTicksPerInch = (kCountsPerRev * kGearRatio) / (2 * Math.PI * kWheelRadiusInches); // 469
  
//   public final static int kEncoderUnitsPerRotation = Double.intValue( (Math.PI * 24.75) * kEncoderTicksPerInch ) );
  public final static int kEncoderUnitsPerRotation = 35000;
  public final static double kEncoderTicksPerDegree = kEncoderUnitsPerRotation / 360;
  private final int kTimeoutMs = 0;
  private final double kNeutralDeadband = 0.002;
  private final double kControllerDeadband = 0.1;
  private final double kTrackWidthMeters = .546;
  private final double kTrackWidthInches = 24.75;
  private final double kRobotMass = 55.3;
  private final int kclosedLoopTimeMs = 1;

	private final double MAX_TELEOP_DRIVE_SPEED = 1.0;
	private final double DRIVE_TRAIN_MM_TOLERANCE = 469;
	// private final double arbFF = 0.2;
	// TalonFX's for the drivetrain
	// Right side is inverted here to drive forward
	WPI_TalonFX m_left_leader, m_right_leader, m_left_follower, m_right_follower;
	// WPI_TalonFX m_left_leader, m_right_leader;//, m_left_follower, m_right_follower;

	// Variables to hold the invert types for the talons
	TalonFXInvertType m_left_invert, m_right_invert;
	
	// Gyro 
	public AHRS m_gyro;
	
	// Auxilliary PID tracker
	// private boolean m_was_correcting = false;
	// private boolean m_is_correcting = false;

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

	// All the Shuffleboard related artifacts
	// Topics to put entries into network tables
	//private final DoubleTopic m_left_encoder_topic, m_right_encoder_topic, m_left_speed_topic, m_right_speed_topic;

	// Entries to periodically update the network table entries
	private final DoubleEntry m_left_encoder_entry, m_right_encoder_entry, m_left_speed_entry, m_right_speed_entry, m_max_speed_entry, kP_entry, kD_entry, kI_entry, kF_entry;

	TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
	TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
	SlotConfiguration left_forward_config = _leftConfig.slot0;
	SlotConfiguration left_reverse_config = _leftConfig.slot1;
	SlotConfiguration right_forward_config = _rightConfig.slot0;
	SlotConfiguration right_reverse_config = _rightConfig.slot1;

	// Motion Magic Setpoints for each side of the motor
	private double m_left_setpoint, m_right_setpoint;

  	/** Creates a new DriveTrain. */
 	public DriveTrain() {
		m_gyro = new AHRS(SPI.Port.kMXP);
		m_left_leader = new WPI_TalonFX(LEFT_TALON_LEADER);
		m_left_follower = new  WPI_TalonFX(LEFT_TALON_FOLLOWER);
		m_right_leader = new WPI_TalonFX(RIGHT_TALON_LEADER);
		m_right_follower = new WPI_TalonFX(RIGHT_TALON_FOLLOWER);

		m_left_leader.configFactoryDefault();
		m_left_follower.configFactoryDefault();
		m_right_leader.configFactoryDefault();
		m_right_follower.configFactoryDefault();

		/** Invert Directions for Left and Right */
		m_left_leader.setInverted(TalonFXInvertType.Clockwise); //Same invert as = "true"
		m_right_leader.setInverted(TalonFXInvertType.CounterClockwise); //Same invert as = "false"

		// Set follower talons to default configs, and then follo–w their leaders
		m_left_follower.follow(m_left_leader);
		m_left_follower.setInverted(InvertType.FollowMaster);
		m_right_follower.follow(m_right_leader);
		m_right_follower.setInverted(InvertType.FollowMaster);

		/* Set Neutral Mode */
		m_left_leader.setNeutralMode(NeutralMode.Brake);
		m_right_leader.setNeutralMode(NeutralMode.Brake);

		/** Config Objects for motor controllers */
		
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

		// Create topics in the Shuffleboard table
		NetworkTableInstance nt_inst = NetworkTableInstance.getDefault();
		NetworkTable nt_table = nt_inst.getTable("Shuffleboard/Drive Test");

		m_right_encoder_entry = new DoubleTopic(nt_table.getDoubleTopic("Right Encoder")).getEntry(0, PubSubOption.keepDuplicates(true));
		m_left_encoder_entry = new DoubleTopic(nt_table.getDoubleTopic("Left Encoder")).getEntry(0, PubSubOption.keepDuplicates(true));
		m_right_speed_entry = new DoubleTopic(nt_table.getDoubleTopic("Right Speed")).getEntry(0, PubSubOption.keepDuplicates(true));
		m_left_speed_entry = new DoubleTopic(nt_table.getDoubleTopic("Left Speed")).getEntry(0, PubSubOption.keepDuplicates(true));
		m_max_speed_entry = new DoubleTopic(nt_table.getDoubleTopic("Max Speed")).getEntry(0, PubSubOption.keepDuplicates(true));

		kP_entry = new DoubleTopic(nt_table.getDoubleTopic("kP")).getEntry(0, PubSubOption.keepDuplicates(true));
		kD_entry = new DoubleTopic(nt_table.getDoubleTopic("kD")).getEntry(0, PubSubOption.keepDuplicates(true));
		kI_entry = new DoubleTopic(nt_table.getDoubleTopic("kI")).getEntry(0, PubSubOption.keepDuplicates(true));
		kF_entry = new DoubleTopic(nt_table.getDoubleTopic("kF")).getEntry(0, PubSubOption.keepDuplicates(true));
		
		//PID slot entries
		left_forward_config.kP = kP_entry.getAsDouble();
		left_forward_config.kD = kD_entry.getAsDouble();
		left_forward_config.kI = kI_entry.getAsDouble();
		left_forward_config.kF = kF_entry.getAsDouble();
		left_forward_config.allowableClosedloopError = 256;
		left_forward_config.closedLoopPeakOutput = MAX_TELEOP_DRIVE_SPEED;
		left_forward_config.closedLoopPeriod = kclosedLoopTimeMs;
		_leftConfig.slot0 = left_forward_config;
		
		left_reverse_config.kP = kP_entry.getAsDouble();
		left_reverse_config.kD = kD_entry.getAsDouble();
		left_reverse_config.kI = kI_entry.getAsDouble();
		left_reverse_config.kF = kF_entry.getAsDouble();
		left_reverse_config.allowableClosedloopError = 256;
		left_reverse_config.closedLoopPeakOutput = MAX_TELEOP_DRIVE_SPEED;
		left_reverse_config.closedLoopPeriod = kclosedLoopTimeMs;
		_leftConfig.slot1 = left_reverse_config;
		
		
		right_forward_config.kP = kP_entry.getAsDouble();
		right_forward_config.kD = kD_entry.getAsDouble();
		right_forward_config.kI = kI_entry.getAsDouble();
		right_forward_config.kF = kF_entry.getAsDouble();
		right_forward_config.allowableClosedloopError = 256;
		right_forward_config.closedLoopPeakOutput = MAX_TELEOP_DRIVE_SPEED;
		right_forward_config.closedLoopPeriod = kclosedLoopTimeMs;
		_rightConfig.slot0 = right_forward_config;

		right_reverse_config.kP = kP_entry.getAsDouble();
		right_reverse_config.kD = kD_entry.getAsDouble();
		right_reverse_config.kI = kI_entry.getAsDouble();
		right_reverse_config.kF = kF_entry.getAsDouble();
		right_reverse_config.allowableClosedloopError = 256;
		right_reverse_config.closedLoopPeakOutput = MAX_TELEOP_DRIVE_SPEED;
		right_reverse_config.closedLoopPeriod = kclosedLoopTimeMs;
		_rightConfig.slot1 = right_reverse_config;

		m_left_leader.configAllSettings(_leftConfig);
		m_right_leader.configAllSettings(_rightConfig);
		
  	}

	@Override
	public void periodic() {
		m_odometry.update(m_gyro.getRotation2d(),
                      nativeUnitsToDistanceMeters(m_left_leader.getSelectedSensorPosition()),
                      nativeUnitsToDistanceMeters(m_right_leader.getSelectedSensorPosition()));
   		m_2dField.setRobotPose(m_odometry.getPoseMeters());

		// Update the encoder topics with timestamps
		m_left_encoder_entry.set(m_left_leader.getSelectedSensorPosition(), 0);
		m_right_encoder_entry.set(m_right_leader.getSelectedSensorPosition(), 0);
		m_right_speed_entry.set(getRightSpeed(), 0);
		m_left_speed_entry.set(getLeftSpeed(), 0);
	}

	public void teleop_drive(double forward, double turn){
		forward = MathUtil.applyDeadband(forward, kControllerDeadband);
		turn = MathUtil.applyDeadband(turn, kControllerDeadband);

		forward = MathUtil.clamp(forward, -m_drive_absMax, m_drive_absMax);
		turn = MathUtil.clamp(turn, -m_drive_absMax, m_drive_absMax);

		// //forward = -m_forward_limiter.calculate(forward) * m_drive_absMax;
		// if(forward != 0 || turn != 0) {
		// 	forward = m_forward_limiter.calculate(forward) * m_drive_absMax;
		// 	turn = m_rotation_limiter.calculate(turn) * m_drive_absMax;
		// }

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
		double motorRotations = wheelRotations * kGearRatio;
		int sensorCounts = (int)(motorRotations * kCountsPerRev);
		return sensorCounts;
	}

	private int velocityToNativeUnits(double velocityMetersPerSecond){
		double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
		double motorRotationsPerSecond = wheelRotationsPerSecond * kGearRatio;
		double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
		int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
		return sensorCountsPer100ms;
	}

	private double nativeUnitsToDistanceMeters(double sensorCounts){
		double motorRotations = (double)sensorCounts / kCountsPerRev;
		double wheelRotations = motorRotations / kGearRatio;
		double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
	  return positionMeters;
	}

  public TalonFXSimCollection getLeftSimCollection(){
    return m_left_leader.getSimCollection();
  }

  public TalonFXSimCollection getRightSimCollection(){
    return m_right_leader.getSimCollection();
  }

  public double getLeftSpeed(){
    return m_left_leader.getSelectedSensorVelocity();
  }

  public double getRightSpeed(){
    return m_right_leader.getSelectedSensorVelocity();
  }

  public DifferentialDrivetrainSim getDriveTrainSim(){
    return m_drivetrainSimulator;
  }

  public void reset_max_speed(){
	m_drive_absMax = m_max_speed_entry.get(0);
  }

  public double get_max_speed(){
	return m_drive_absMax;
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase setMaxValue() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
		  m_drive_absMax = m_max_speed_entry.get(0);
        });
  }

  public void configureLeftForwardPidSlot() {
	left_forward_config.kP = kP_entry.getAsDouble();
	left_forward_config.kD = kD_entry.getAsDouble();
	left_forward_config.kI = kI_entry.getAsDouble();
	left_forward_config.kF = kF_entry.getAsDouble();
	left_forward_config.allowableClosedloopError = 256;
	left_forward_config.closedLoopPeakOutput = MAX_TELEOP_DRIVE_SPEED;
	left_forward_config.closedLoopPeriod = kclosedLoopTimeMs;
	_leftConfig.slot0 = left_forward_config;
  }
  
  public void configureLeftReversePidSlot() {
	left_reverse_config.kP = kP_entry.getAsDouble();
	left_reverse_config.kD = kD_entry.getAsDouble();
	left_reverse_config.kI = kI_entry.getAsDouble();
	left_reverse_config.kF = kF_entry.getAsDouble();
	left_reverse_config.allowableClosedloopError = 256;
	left_reverse_config.closedLoopPeakOutput = MAX_TELEOP_DRIVE_SPEED;
	left_reverse_config.closedLoopPeriod = kclosedLoopTimeMs;
	_leftConfig.slot1 = left_reverse_config;
  }
  
  public void configureRightForwardPidSlot() {
	right_forward_config.kP = kP_entry.getAsDouble();
	right_forward_config.kD = kD_entry.getAsDouble();
	right_forward_config.kI = kI_entry.getAsDouble();
	right_forward_config.kF = kF_entry.getAsDouble();
	right_forward_config.allowableClosedloopError = 256;
	right_forward_config.closedLoopPeakOutput = MAX_TELEOP_DRIVE_SPEED;
	right_forward_config.closedLoopPeriod = kclosedLoopTimeMs;
	_rightConfig.slot0 = right_forward_config;
  }
  
  public void configureRightReversePidSlot() {
	right_reverse_config.kP = kP_entry.getAsDouble();
	right_reverse_config.kD = kD_entry.getAsDouble();
	right_reverse_config.kI = kI_entry.getAsDouble();
	right_reverse_config.kF = kF_entry.getAsDouble();
	right_reverse_config.allowableClosedloopError = 256;
	right_reverse_config.closedLoopPeakOutput = MAX_TELEOP_DRIVE_SPEED;
	right_reverse_config.closedLoopPeriod = kclosedLoopTimeMs;
	_rightConfig.slot1 = right_reverse_config;
  }

  public void configureMotionMagic(int target) {
    double left_currentPosition = m_left_leader.getSelectedSensorPosition();
    double right_currentPosition = m_right_leader.getSelectedSensorPosition();

    if(left_currentPosition < target) {
      configureLeftForwardPidSlot();
    } else if(left_currentPosition > target) {
      configureLeftReversePidSlot();
    } else {
      //does nothing
	}

	if(right_currentPosition < target){
	  configureRightForwardPidSlot();
	} else if (left_currentPosition > target){
	  configureRightReversePidSlot();
	} else {
	  //does nothing 
	}
}
	
  public boolean driveMotionMagic(int target) {
    m_right_leader.set(ControlMode.MotionMagic, target);
	m_left_leader.set(ControlMode.MotionMagic, target);

    double right_error = Math.abs(m_right_leader.getClosedLoopError());
	double left_error = Math.abs(m_left_leader.getClosedLoopError());

    return right_error <= DRIVE_TRAIN_MM_TOLERANCE && left_error <= DRIVE_TRAIN_MM_TOLERANCE;
  }

  public boolean motionMagicTurn(double arcTicks){
	double tolerance = 75;
	if(arcTicks > 0){
		m_left_leader.set(ControlMode.MotionMagic, m_left_setpoint);
		m_right_leader.set(ControlMode.MotionMagic, m_right_setpoint);
	}else{
		m_left_leader.set(ControlMode.MotionMagic, m_left_setpoint);
		m_right_leader.set(ControlMode.MotionMagic, m_right_setpoint);
	}
	double currentLeftPos =  m_left_leader.getSelectedSensorPosition();
	double currentRightPos = m_right_leader.getSelectedSensorPosition();
	 
	return Math.abs(m_left_setpoint - currentLeftPos) < tolerance && Math.abs(m_right_setpoint - currentRightPos) < tolerance;

}

public void motion_magic_start_config_drive(boolean isForward, double lengthInTicks){
	m_left_setpoint = m_left_leader.getSelectedSensorPosition() + lengthInTicks;
	m_right_setpoint = m_right_leader.getSelectedSensorPosition() + lengthInTicks;

	m_left_leader.configMotionCruiseVelocity(16636,kTimeoutMs);
	m_left_leader.configMotionAcceleration(8318, kTimeoutMs); //cruise velocity / 2, so will take 2 seconds
	m_right_leader.configMotionCruiseVelocity(16636,kTimeoutMs);
	m_right_leader.configMotionAcceleration(8318, kTimeoutMs);
	
	//set up talon to use DriveMM slots
	m_left_leader.selectProfileSlot(kSlot_DriveMM, PID_PRIMARY);
	m_right_leader.selectProfileSlot(kSlot_DriveMM, PID_PRIMARY);
}

public void reset_drive_PID_values(double kP, double kI, double kD) {
	m_left_leader.config_kP(kSlot_DriveMM, kP);
	m_left_leader.config_kI(kSlot_DriveMM, kI);
	m_left_leader.config_kD(kSlot_DriveMM, kD);
	
	m_right_leader.config_kP(kSlot_DriveMM, kP);
	m_right_leader.config_kI(kSlot_DriveMM, kI);
	m_right_leader.config_kD(kSlot_DriveMM, kD); 
}
// public void motionMagicStartConfigsTurn(boolean isCCWturn, double lengthInTicks){

// 	m_left_leader.selectProfileSlot(kSlot_Turning);
// 	m_right_leader.selectProfileSlot(kSlot_Turning);
// 	m_left_leader.configMotionCruiseVelocity(16636, kTimeoutMs);
// 	m_left_leader.configMotionAcceleration(4159, kTimeoutMs);
// 	m_right_leader.configMotionCruiseVelocity(16636, kTimeoutMs);
// 	m_right_leader.configMotionAcceleration(4159, kTimeoutMs);

// 	// length in Ticks is negative
// 	m_left_setpoint = m_left_leader.getSelectedSensorPosition() + lengthInTicks;
// 	m_right_setpoint = m_right_leader.getSelectedSensorPosition() - lengthInTicks;
// }
}
