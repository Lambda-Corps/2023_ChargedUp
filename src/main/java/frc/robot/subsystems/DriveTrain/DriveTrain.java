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
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
	 * Using the configSelectedFeedbackCoefficient() function, scale units to 3600
	 * per rotation.
	 * This is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
	 */
	public final static double kTurnTravelUnitsPerRotation = 3600;

	/**
	 * Empirically measure what the difference between encoders per 360'
	 * Drive the robot in clockwise rotations and measure the units per rotation.
	 * Drive the robot in counter clockwise rotations and measure the units per
	 * rotation.
	 * Take the average of the two.
	 */
	public static final int kCountsPerRev = 2048; // Encoder counts per revolution of the motor shaft.
	public static final double kHighGearRatio = 4.17; // Switch kSensorGearRatio to this gear ratio if encoder is on the
														// motor instead
	public static final double kLowGearRatio = 11.03;
	// of on the gearbox.
	public static final double kWheelRadiusInches = 2.9;
	public static final int k100msPerSecond = 10;
	public static final double kEncoderTicksPerInch = (kCountsPerRev * kLowGearRatio)
			/ (2 * Math.PI * kWheelRadiusInches); // 469

	// public final static int kEncoderUnitsPerRotation = Double.intValue( (Math.PI
	// * 24.75) * kEncoderTicksPerInch ) );
	public final static int kEncoderUnitsPerRotation = 35000;
	public final static double kEncoderTicksPerDegree = kEncoderUnitsPerRotation / 360;
	private final double kNeutralDeadband = 0.005;
	private final double kControllerDeadband = 0.1;
	private final double kTrackWidthMeters = .546;
	// private final double kTrackWidthInches = 24.75;
	private final double kRobotMass = 55.3;
	private final int kTimeoutMs = 10;

	private final double MAX_TELEOP_DRIVE_SPEED = 1.0;
	// private final double arbFF = 0.075;
	// Fine grained driving will square the inputs, so .6 will really end up being .36 max driving when 
	// the fine grained control is being applied.
	private final double FINE_GRAINED_MAX = .6; 
	private final double TURN_WITH_GYRO_KP = .015;

	private final DoubleSolenoid.Value HIGH_GEAR = DoubleSolenoid.Value.kForward;
	private final DoubleSolenoid.Value LOW_GEAR = DoubleSolenoid.Value.kReverse;

	// TalonFX's for the drivetrain
	// Right side is inverted here to drive forward
	WPI_TalonFX m_left_leader, m_right_leader, m_left_follower, m_right_follower;
	// WPI_TalonFX m_left_leader, m_right_leader;//, m_left_follower,
	// m_right_follower;

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
	// private final DoubleTopic m_left_encoder_topic, m_right_encoder_topic,
	// m_left_speed_topic, m_right_speed_topic;

	// Entries to periodically update the network table entries
	private final DoubleEntry m_left_encoder_entry, m_right_encoder_entry, m_left_speed_entry, m_right_speed_entry,
			m_max_speed_entry;

	final double MM_DRIVE_KP = 0.495;
	final double MM_DRIVE_KD = 4.9;
	final int MM_SLOT = 0;
	final int PID_PRIMARY = 0;
	final int MM_TOLERANCE = 200;
	final int MM_VELOCITY = 8000;
	final int MM_ACCELERATION = 8000;
	final int FORWARD_SLEW_RATE = 3;
	final int TURN_SLEW_RATE = 5;
	double TURN_DRIVE_FF = .1;
	final double DRIVE_BANG_BANG_FWD = .3;
	final double DRIVE_BANG_BANG_BACK = -.2;
	final int DRIVE_BANG_BANG_SP = 10;
	double DRIVE_STRAIGHT_FF = .2;
	int m_setpoint_left = 0;
	int m_setpoint_right = 0;
	double m_turn_setpoint = 0;

	DoubleSolenoid m_shifter;

	PIDController m_turn_pid_controller, m_drive_pid_controller;

	BangBangController m_forward_bang_bang, m_reverse_bang_bang;

	/** Creates a new DriveTrain. */
	public DriveTrain() {
		m_gyro = new AHRS(SPI.Port.kMXP);
		m_left_leader = new WPI_TalonFX(LEFT_TALON_LEADER);
		m_left_follower = new WPI_TalonFX(LEFT_TALON_FOLLOWER);
		m_right_leader = new WPI_TalonFX(RIGHT_TALON_LEADER);
		m_right_follower = new WPI_TalonFX(RIGHT_TALON_FOLLOWER);

		m_left_leader.configFactoryDefault();
		m_left_follower.configFactoryDefault();
		m_right_leader.configFactoryDefault();
		m_right_follower.configFactoryDefault();

		// Configure Motion Magic constants
		TalonFXConfiguration talon_config = new TalonFXConfiguration();
		talon_config.slot0.kP = MM_DRIVE_KP;
		talon_config.slot0.kD = MM_DRIVE_KD;
		talon_config.slot0.allowableClosedloopError = 25;
		talon_config.slot0.closedLoopPeriod = 1;
		talon_config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
		talon_config.neutralDeadband = kNeutralDeadband;
		talon_config.motionCruiseVelocity = MM_VELOCITY;
		talon_config.motionAcceleration = MM_ACCELERATION;

		if( Robot.isSimulation() ){
			// The simulator needs much smaller motion magic values due to not 
			// being real physics
			talon_config.slot0.kP = 0.1;
			talon_config.motionCruiseVelocity = 5000;
			talon_config.motionAcceleration = 5000;
		}

		m_left_leader.configAllSettings(talon_config);
		m_right_leader.configAllSettings(talon_config);

		// Default the controllers to use the primary slots for MotionMagic
		m_left_leader.selectProfileSlot(MM_SLOT, PID_PRIMARY);
		m_right_leader.selectProfileSlot(MM_SLOT, PID_PRIMARY);

		/** Invert Directions for Left and Right */
		m_left_leader.setInverted(TalonFXInvertType.Clockwise); // Same invert as = "true"
		m_right_leader.setInverted(TalonFXInvertType.CounterClockwise); // Same invert as = "false"

		// Set follower talons to default configs, and then follo–w their leaders
		m_left_follower.follow(m_left_leader);
		m_left_follower.setInverted(InvertType.FollowMaster);
		m_right_follower.follow(m_right_leader);
		m_right_follower.setInverted(InvertType.FollowMaster);

		/* Set Neutral Mode */
		m_left_leader.setNeutralMode(NeutralMode.Brake);
		m_right_leader.setNeutralMode(NeutralMode.Brake);

		/** Config Objects for motor controllers */
		// TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
		// TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

		m_left_leader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_PRIMARY, kTimeoutMs);
		m_right_leader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_PRIMARY, kTimeoutMs);

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
					kLowGearRatio, // Standard AndyMark Gearing reduction.
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
		m_forward_limiter = new SlewRateLimiter(FORWARD_SLEW_RATE);
		m_rotation_limiter = new SlewRateLimiter(TURN_SLEW_RATE);
		m_drive_absMax = MAX_TELEOP_DRIVE_SPEED;

		m_gyro.reset();

		// Create topics in the Shuffleboard table
		NetworkTableInstance nt_inst = NetworkTableInstance.getDefault();
		NetworkTable nt_table = nt_inst.getTable("Shuffleboard/Drive Test");

		m_right_encoder_entry = new DoubleTopic(nt_table.getDoubleTopic("Right Encoder")).getEntry(0,
				PubSubOption.keepDuplicates(true));
		m_left_encoder_entry = new DoubleTopic(nt_table.getDoubleTopic("Left Encoder")).getEntry(0,
				PubSubOption.keepDuplicates(true));
		m_right_speed_entry = new DoubleTopic(nt_table.getDoubleTopic("Right Speed")).getEntry(0,
				PubSubOption.keepDuplicates(true));
		m_left_speed_entry = new DoubleTopic(nt_table.getDoubleTopic("Left Speed")).getEntry(0,
				PubSubOption.keepDuplicates(true));
		m_max_speed_entry = new DoubleTopic(nt_table.getDoubleTopic("Max Speed")).getEntry(0,
				PubSubOption.keepDuplicates(true));

		m_shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
		m_shifter.set(LOW_GEAR);

		m_turn_pid_controller = new PIDController(TURN_WITH_GYRO_KP, 0, 0);

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

		m_forward_bang_bang = new BangBangController();
		m_forward_bang_bang.setSetpoint(DRIVE_BANG_BANG_SP);
		m_reverse_bang_bang = new BangBangController();
		m_reverse_bang_bang.setSetpoint(-DRIVE_BANG_BANG_SP);
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

	public void teleop_drive(double forward, double turn) {
		forward = MathUtil.applyDeadband(forward, kControllerDeadband);
		turn = MathUtil.applyDeadband(turn, kControllerDeadband);

		forward = MathUtil.clamp(forward, -m_drive_absMax, m_drive_absMax);
		turn = MathUtil.clamp(turn, -m_drive_absMax, m_drive_absMax);

		// forward = -m_forward_limiter.calculate(forward) * m_drive_absMax;
		if (forward != 0 || turn != 0) {
			forward = m_forward_limiter.calculate(forward) * m_drive_absMax;
			turn = m_rotation_limiter.calculate(turn) * m_drive_absMax;
		} else {
			if (forward != 0){
				m_forward_limiter.reset(FORWARD_SLEW_RATE);
			}
			if( turn != 0 ){
				m_rotation_limiter.reset(TURN_SLEW_RATE);
				
			}
		}

		var speeds = DifferentialDrive.curvatureDriveIK(forward, turn, true);

		if (Robot.isSimulation()) {
			// Just set the motors
			m_right_leader.set(ControlMode.PercentOutput, speeds.right);
			m_left_leader.set(ControlMode.PercentOutput, speeds.left);
			return;
		}
		// Just set the motors
		m_right_leader.set(ControlMode.PercentOutput, speeds.right);
		m_left_leader.set(ControlMode.PercentOutput, speeds.left);
	}

	public void fine_grained_drive(double forward, double turn) {
		forward = MathUtil.applyDeadband(forward, kControllerDeadband);
		turn = MathUtil.applyDeadband(turn, kControllerDeadband);

		forward = MathUtil.clamp(forward, -FINE_GRAINED_MAX, FINE_GRAINED_MAX);
		turn = MathUtil.clamp(turn, -FINE_GRAINED_MAX, FINE_GRAINED_MAX);

		// forward = -m_forward_limiter.calculate(forward) * m_drive_absMax;
		if (forward != 0 || turn != 0) {
			forward = m_forward_limiter.calculate(forward) * m_drive_absMax;
			turn = m_rotation_limiter.calculate(turn) * m_drive_absMax;
		} else {
			if (forward != 0){
				m_forward_limiter.reset(FORWARD_SLEW_RATE);
			}
			if( turn != 0 ){
				m_rotation_limiter.reset(TURN_SLEW_RATE);
				
			}
		}

		var speeds = DifferentialDrive.arcadeDriveIK(forward, turn, true);

		if (Robot.isSimulation()) {
			// Just set the motors
			m_right_leader.set(ControlMode.PercentOutput, speeds.right);
			m_left_leader.set(ControlMode.PercentOutput, speeds.left);
			return;
		}
		// Just set the motors
		m_right_leader.set(ControlMode.PercentOutput, speeds.right);
		m_left_leader.set(ControlMode.PercentOutput, speeds.left);
	}

	public void turn_ccw_positive(double turn){
		// turn = MathUtil.applyDeadband(turn, kControllerDeadband);

		if( Robot.isReal() ){
			// Make sure we don't have output that is too high
			turn = MathUtil.clamp(turn, -.3, .3);

			// If we're too low, add the FF to make sure the robot keeps spinning
			if( turn > 0 ){
				if(turn < TURN_DRIVE_FF){
					turn = TURN_DRIVE_FF;
				}
			}else if( turn < 0 ) {
				if( turn > -TURN_DRIVE_FF){
					turn = -TURN_DRIVE_FF;
				}
			}
		} else {
			// Simulation doesn't have real physics, just limit the turn speed to 
			// 20% arbitrarily
			turn = MathUtil.clamp(turn, -.1, .1);
		}

		// Set the motors, CCW means that the left side will go backward
		// and the right will go forward if we are turning positive
		m_right_leader.set(ControlMode.PercentOutput, turn);
		m_left_leader.set(ControlMode.PercentOutput, -turn);
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
		m_drivetrainSimulator.setInputs(-m_leftDriveSim.getMotorOutputLeadVoltage(),
				m_rightDriveSim.getMotorOutputLeadVoltage());

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
						-m_drivetrainSimulator.getLeftPositionMeters()));
		m_leftDriveSim.setIntegratedSensorVelocity(
				velocityToNativeUnits(
						-m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
		m_rightDriveSim.setIntegratedSensorRawPosition(
				distanceToNativeUnits(
						m_drivetrainSimulator.getRightPositionMeters()));
		m_rightDriveSim.setIntegratedSensorVelocity(
				velocityToNativeUnits(
						m_drivetrainSimulator.getRightVelocityMetersPerSecond()));

		int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
		SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
		angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
	}

	private int distanceToNativeUnits(double positionMeters) {
		double wheelRotations = positionMeters / (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
		double motorRotations = wheelRotations * kLowGearRatio;
		int sensorCounts = (int) (motorRotations * kCountsPerRev);
		return sensorCounts;
	}

	private int velocityToNativeUnits(double velocityMetersPerSecond) {
		double wheelRotationsPerSecond = velocityMetersPerSecond
				/ (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
		double motorRotationsPerSecond = wheelRotationsPerSecond * kLowGearRatio;
		double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
		int sensorCountsPer100ms = (int) (motorRotationsPer100ms * kCountsPerRev);
		return sensorCountsPer100ms;
	}

	private double nativeUnitsToDistanceMeters(double sensorCounts) {
		double motorRotations = (double) sensorCounts / kCountsPerRev;
		double wheelRotations = motorRotations / kLowGearRatio;
		double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
		return positionMeters;
	}

	public TalonFXSimCollection getLeftSimCollection() {
		return m_left_leader.getSimCollection();
	}

	public TalonFXSimCollection getRightSimCollection() {
		return m_right_leader.getSimCollection();
	}

	public double getLeftSpeed() {
		return m_left_leader.getSelectedSensorVelocity();
	}

	public double getRightSpeed() {
		return m_right_leader.getSelectedSensorVelocity();
	}

	public DifferentialDrivetrainSim getDriveTrainSim() {
		return m_drivetrainSimulator;
	}

	public void reset_max_speed() {
		m_drive_absMax = m_max_speed_entry.get(0);
	}

	public double get_max_speed() {
		return m_drive_absMax;
	}

	// Get the robot's heading in degrees scaled to 360
	// Output is negated because the Navx is Clockwise positive, but
	// our robot is NWU orientation, nor CounterClockWise positive.
	public double getScaledHeading() {
		return Math.IEEEremainder(-m_gyro.getAngle(), 360);
	}

	public double getCCWPositiveHeading(){
		return -m_gyro.getAngle();
	}

	public double getYaw(){
		return m_gyro.getYaw();
	}

	public double getPitch(){
		return m_gyro.getPitch();
	}

	public double getRoll(){
		return m_gyro.getRoll();
	}

	public double getAngle(){
		return m_gyro.getAngle();
	}

	public void configurePIDTurn(double kP, double kD, double kI, double setpoint){
		m_turn_pid_controller = new PIDController(kP, kI, kD);
		
		m_turn_pid_controller.setTolerance(1);
		// m_turn_pid_controller.enableContinuousInput(-180, 180);
		m_turn_pid_controller.setSetpoint(setpoint);
	}

	public void configurePIDDrive(double kP, double kD, double kI, double setpoint){
		m_drive_pid_controller = new PIDController(kP, kI, kD);
		
		m_drive_pid_controller.setTolerance(100);
		// m_turn_pid_controller.enableContinuousInput(-180, 180);
		m_drive_pid_controller.setSetpoint(setpoint);
		m_setpoint_left = (int)setpoint;
		SmartDashboard.putData(m_drive_pid_controller);
	}

	public void set_turn_target_setpoint(double angle_setpoint){
		m_turn_setpoint = getCCWPositiveHeading() + angle_setpoint;
		m_turn_pid_controller.setTolerance(1);
	}

	public boolean turn_target_degrees() {
		// Calculate the error between our setpoint and current angle
		// The Navx is reversed, so used CCWPositiveHeading which negates the output
		double turn_output = m_turn_pid_controller.calculate(getCCWPositiveHeading(), m_turn_setpoint);

		// Use this custom drive method to turn CCW positive
		turn_ccw_positive(turn_output);

		return m_turn_pid_controller.atSetpoint();
	}

	public boolean drive_straight_with_pid() {
		// Calculate the error between our setpoint and current angle
		// The Navx is reversed, so used CCWPositiveHeading which negates the output
		double forward_output = m_drive_pid_controller.calculate(m_left_leader.getSelectedSensorPosition(), m_setpoint_left);
		if( forward_output > 0 ){
			if( forward_output < DRIVE_STRAIGHT_FF){
				forward_output = DRIVE_STRAIGHT_FF;
			}
		} else if (forward_output < 0 ){
			if (forward_output > -DRIVE_STRAIGHT_FF ){
				forward_output = -DRIVE_STRAIGHT_FF;
			}
		}

		forward_output = MathUtil.clamp(forward_output, -.7, .7);
		forward_output = Math.copySign(forward_output * forward_output, forward_output);

		forward_output = m_forward_limiter.calculate(forward_output);
		// Use this custom drive method to turn CCW positive
		teleop_drive(forward_output,0);

		return m_drive_pid_controller.atSetpoint();
	}

	public AHRS getGyro(){
		return m_gyro;
	}

	public PIDController get_dt_turn_pidcontroller(){
		return m_turn_pid_controller;
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

	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public CommandBase stopMotorsCommand() {
		// Inline construction of command goes here.
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(
				() -> {
					/* one-time action goes here */
					m_left_leader.set(ControlMode.PercentOutput, 0);
					m_right_leader.set(ControlMode.PercentOutput, 0);
				});
	}

	public void configure_motion_magic_test(double velocity, double time_to_velo, double kp) {
		// Dividing by zero is very bad, will crash most systems. 
		if( time_to_velo == 0 ){
			time_to_velo = 1;
		}
		double acceleration = velocity / time_to_velo;

		m_left_leader.selectProfileSlot(MM_SLOT, PID_PRIMARY);
		m_right_leader.selectProfileSlot(MM_SLOT, PID_PRIMARY);

		m_left_leader.config_kP(MM_SLOT, kp);
		m_right_leader.config_kP(MM_SLOT, kp);
		m_left_leader.configMotionCruiseVelocity(velocity);
		m_right_leader.configMotionCruiseVelocity(velocity);
		m_left_leader.configMotionAcceleration(acceleration);
		m_right_leader.configMotionAcceleration(acceleration);

		m_left_leader.configAllowableClosedloopError(MM_SLOT, 10);
		m_right_leader.configAllowableClosedloopError(MM_SLOT, 10);
	}

	public void configure_motion_magic(int setpoint) {
		m_left_leader.selectProfileSlot(MM_SLOT, PID_PRIMARY);
		m_right_leader.selectProfileSlot(MM_SLOT, PID_PRIMARY);

		m_setpoint_left = (int)(m_left_leader.getSelectedSensorPosition() + setpoint);
		m_setpoint_right = (int)(m_right_leader.getSelectedSensorPosition() + setpoint);
	}

	public void reset_setpoints() {
		m_setpoint_left = 0;
		m_setpoint_right = 0;
	}

	public double get_setpoint() {
		return m_setpoint_left;
	}

	public boolean drive_motion_magic() {
		// boolean done;
		m_left_leader.set(ControlMode.MotionMagic, m_setpoint_left);
		m_right_leader.set(ControlMode.MotionMagic, m_setpoint_right);
		// m_right_leader.set(ControlMode.MotionMagic, m_setpoint, DemandType.ArbitraryFeedForward, arbFF);

		// double currentPos_L = m_left_leader.getSelectedSensorPosition();
		// double currentPos_R = m_right_leader.getSelectedSensorPosition();

		// // boolean left_done = m_left_leader.getClosedLoopError() < MM_TOLERANCE;
		// // boolean right_done = m_right_leader.getClosedLoopError() < MM_TOLERANCE;
		// boolean left_done = Math.abs((m_setpoint - currentPos_L)) < MM_TOLERANCE;
		// boolean right_done = Math.abs((m_setpoint - currentPos_R)) < MM_TOLERANCE;

		// return left_done && right_done;
		return false;
	}

	public boolean is_drive_mm_done() {
		boolean done = false;

		double currentPos_L = m_left_leader.getSelectedSensorPosition();
		double currentPos_R = m_right_leader.getSelectedSensorPosition();

		// boolean left_done = m_left_leader.getClosedLoopError() < MM_TOLERANCE;
		// boolean right_done = m_right_leader.getClosedLoopError() < MM_TOLERANCE;
		boolean left_done = Math.abs((m_setpoint_left - currentPos_L)) <  MM_TOLERANCE;
		boolean right_done = Math.abs(m_setpoint_right - currentPos_R) < MM_TOLERANCE;

		done = left_done && right_done;

		return done;
	}

	public double getLeftError() {
		return m_left_leader.getClosedLoopError();
	}

	public double getRightError() {
		return m_right_leader.getClosedLoopError();
	}

	public void reset_encoders() {
		m_right_leader.setSelectedSensorPosition(0,0,0);
		m_left_leader.setSelectedSensorPosition(0,0,0);
	}

	public boolean at_pid_setpoint(){
		return m_turn_pid_controller.atSetpoint();
	}

	public void configure_forward_bangbang_controller(double setpoint) {
		m_forward_bang_bang.setSetpoint(setpoint);
	}

	public void configure_reverse_bangbang_controller(double setpoint) {
		m_reverse_bang_bang.setSetpoint(setpoint);
	}

	public boolean is_fwd_bangbang_at_setpoint() {
		return m_forward_bang_bang.atSetpoint();
	}

	public boolean is_rev_bangbang_at_setpoint() {
		return m_reverse_bang_bang.atSetpoint();
	}

	public double calc_fwd_bangbang() {
		return m_forward_bang_bang.calculate(m_gyro.getRoll());
	}

	public double calc_rev_bangbang() {
		return m_reverse_bang_bang.calculate(m_gyro.getRoll());
	}

	public double get_fwd_bang_bang(double measurement) {
		double ret = 0;

		// 1 if > setpoint
		if( m_forward_bang_bang.calculate(measurement) == 0) {
			ret = 1;
		}

		return ret;
	}
	
	public double get_rev_bang_bang(double measurement) {
		return m_reverse_bang_bang.calculate(measurement);
	}

	public double drive_bang_bang() {
		double roll = m_gyro.getRoll();
		
		double speed = (DRIVE_BANG_BANG_FWD * get_fwd_bang_bang(roll)) + 
					   (DRIVE_BANG_BANG_BACK * get_rev_bang_bang(roll));
		// teleop_drive(speed, 0);
		
		teleop_drive(speed, 0);

		return speed;
	}

	// INLINE COMMANDS
	public CommandBase shiftToHighGear() {
		return runOnce(
				() -> {
					m_shifter.set(HIGH_GEAR);
				});
	}

	public CommandBase shiftToLowGear() {
		// Inline construction of command goes here.
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(
				() -> {
					/* one-time action goes here */
					m_shifter.set(LOW_GEAR);
				});
	}

	public CommandBase reset_dt_setpoint() {
		return runOnce(
			() -> {
				reset_setpoints();
			}
		);
	}

	public CommandBase driveSlowlyUntil(){
		return run(
			() -> {
				double speed = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Test").getEntry("Max Speed").getDouble(0);
				System.out.println("Speed: " + speed);
				teleop_drive(speed, 0);
			});
	}

	public CommandBase driveMotionMagic(double target_in_inches){
		return runOnce(
			() -> {
				// Configure the target in encoder ticks, set the motor controllers up
				// and have them go until the setpoint
				int target_in_ticks = (int)(target_in_inches * kEncoderTicksPerInch);

				configure_motion_magic(target_in_ticks);

				drive_motion_magic();
			});
	}

	// This command takes in an (x,y) coordinate pair and sets the robot odometry to match
	// that value.  It will set the integrated encoders to that value in the Talons and 
	// set the gyro angle as well
	//
	// The robot is going to be set on the field up against the scoring elements, so use that
	// as the initial starting assumption.  The field map assumes that the bottom left is the
	// origin in 2d space, but that includes the grid where we can't drive.  So our initial 
	// encoder values are going to be non-zero.
	public CommandBase setRobotStartingPose(double x, double y, double heading){
		return runOnce(
			() -> {
				double encoder_val = distanceToNativeUnits(x);
				m_left_leader.setSelectedSensorPosition(encoder_val);
				m_right_leader.setSelectedSensorPosition(encoder_val);

				m_gyro.setAngleAdjustment(heading);

				Rotation2d rot2d = m_gyro.getRotation2d();
				Pose2d pose = new Pose2d(x, y, rot2d);

				m_odometry.resetPosition(m_gyro.getRotation2d(), x, y,pose);
			});
	}
}
