// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  WPI_TalonFX m_arm_motor, m_wrist_motor;
  DigitalInput m_arm_forward_limit, m_arm_reverse_limit, m_wrist_reverse_limit, m_wrist_forward_limit;
  DoublePublisher m_arm_position, m_wrist_position, m_wrist_motor_rev, m_arm_motor_rev, m_arm_mm_error, m_wrist_mm_error;
  StringPublisher m_super_position;

  private ArmState m_arm_state;
  private ArmControlMode m_arm_control_mode;
  private SuperStructurePosition m_current_position, m_requested_position;
  private ArmTask arm_task;
  private HashMap<SuperStructurePosition, ArrayList<SuperStructurePosition>> illegal_transitions = new HashMap<SuperStructurePosition, ArrayList<SuperStructurePosition>>();
  private ArrayList<SuperStructurePosition> stowed_illegal_transitions = new ArrayList<SuperStructurePosition>();
  private ArrayList<SuperStructurePosition> ground_pickup_illegal_transitions = new ArrayList<SuperStructurePosition>();

  public enum ArmState {
    Moving,
    Holding,
    Inactive,
  }

  public enum ArmControlMode { // Mostly for debugging information, could be used as an arm safety measure in
                               // emergencies
    Manual,
    Automatic,
  }

  public enum SuperStructurePosition {
    Stowed(ARM_STOW, WRIST_STOW){
      @Override
      public String toString() {
          return "Stowed";
      }
    },
    GroundPickup(ARM_GROUND_PICKUP, WRIST_GROUND_PICKUP){
      @Override
      public String toString() {
          return "Ground Pickup";
      }
    },
    SubstationPickup(ARM_SUBSTATION, WRIST_SUBSTATION){
      @Override
      public String toString() {
          return "Substation";
      }
    },
    ScoreLow(ARM_SCORE_LOW, WRIST_SCORE_LOW){
      @Override
      public String toString() {
          return "Score Low";
      }},
    ScoreConeMid(ARM_CONE_MID, WRIST_CONE_MID){
      @Override
      public String toString() {
          return "Cone Mid";
      }
    },
    ScoreConeHigh(ARM_CONE_HIGH, WRIST_CONE_HIGH){
      @Override
      public String toString() {
          return "Cone High";
      }
    },
    ScoreCubeMid(ARM_CUBE_MID, WRIST_CUBE_MID){
      @Override
      public String toString() {
          return "Cube Mid";
      }
    },
    ScoreCubeHigh(ARM_CUBE_HIGH, WRIST_CUBE_HIGH){
      @Override
      public String toString() {
          return "Cube High";
      }
    },
    Manual(1, 1){
      @Override
      public String toString() {
          return "Manual";
      }
    }; // Default small values to make sure calculations won't fail

    private int arm_position, wrist_position;
    private ArrayList<SuperStructurePosition> illegal_transitions;

    private SuperStructurePosition(int arm_pos, int wrist_pos) {
      arm_position = arm_pos;
      wrist_position = wrist_pos;

      illegal_transitions = new ArrayList<SuperStructurePosition>();
    }

    private SuperStructurePosition(int arm_pos, int wrist_pos, SuperStructurePosition illegalPosition) {
      arm_position = arm_pos;
      wrist_position = wrist_pos;

      illegal_transitions = new ArrayList<SuperStructurePosition>();
      illegal_transitions.add(illegalPosition);
    }

    public int getArmPosition(){
      return this.arm_position;
    }

    public int getWristPosition(){
      return this.wrist_position;
    }
  }

  public enum ArmTask {
    HoldPosition,
    MoveToPosition,
    MoveManually,
    Stop,
  }

  ///////// Constants ///////////////
  // Constant values for ARM movement, must be researched and tuned via tuner
  final double ARM_FORWARD_SPEED = .4;
  final double ARM_REVERSE_SPEED = -.4;
  final double WRIST_FORWARD_SPEED = .7;
  final double WRIST_REVERSE_SPEED = -.25;
  final double WRIST_FORWARD_COSINE_FF = .09; // When arm is horizontal, calculation should be 1 * .07
  final double ARM_GEAR_RATIO = 10 * 4 * 4;
  final double WRIST_GEAR_RATIO = 7 * 5 * 4;
  final int WRIST_REVERSE_SOFT_LIMIT = -1000;
  final int WRIST_FORWARD_SOFT_LIMIT = 58000;
  final int ARM_REVERSE_SOFT_LIMIT = 0;
  final int ARM_FORWARD_SOFT_LIMIT = 50000;
  final int SAFE__MOVE_WRIST_POSITION = 10000; // Puts the wrist up at 11 degrees
  // final int ARM_FORWARD_SOFT_LIMIT = (int)(2048 * ARM_GEAR_RATIO * 1/6); // 60
  // degrees rotation
  final double WRIST_MAX_STATOR_CURRENT = 30;
  final double ARM_MAX_STATOR_CURRENT = 30;
  final int ARM_MM_SLOT = 0;
  final int ARM_HOLD_POSITION_SLOT = 2;
  final int WRIST_MM_FORWARD_SLOT = 0;
  final int WRIST_MM_REVERSE_SLOT = 1;
  final int WRIST_HOLD_POSITION_SLOT = 2;



  // The wrist travels 90 degrees total, for manual steps try to go 3 degrees at a
  // time
  final int WRIST_POSITION_STEP = (int) (WRIST_FORWARD_SOFT_LIMIT / 30);
  // The arm travels 46 degrees total, for manual steps try one degree at a time
  final int ARM_POSITION_STEP = (int) (ARM_FORWARD_SOFT_LIMIT / 23);

  /*
   * KP Calculations are:
   * Desired Percent * full throttle / error
   * That means, we want to saturate the feedback (full recovery) with an error
   * value
   */
  final double ARM_MM_KP = (ARM_FORWARD_SPEED * 1023) / 2048; // Tuned manually (ARM_FORWARD_SPEED * 1023) / 2048;
  final double ARM_MM_KI = 0;
  final double ARM_MM_KD = 0;
  final double ARM_MM_KF = 0.051; // (.4 * 1023) / 8000
  final double ARM_MM_FF = 0;
  final int ARM_MM_VELOCITY = 8000;
  final int ARM_MM_ACCELERATION = (int)(ARM_MM_VELOCITY / 1); // 1 Second to full velocity
  final double ARM_HOLD_POSITION_KP = (ARM_FORWARD_SPEED * 1023) / 512; // Tuned manually (ARM_FORWARD_SPEED * 1023) / 2048;
  final double ARM_HOLD_POSITION_KI = 0;
  final double ARM_HOLD_POSITION_KD = 0;
  final double ARM_HOLD_POSITION_KF = 0;
  final double WRIST_MM_FORWARD_KP = (WRIST_FORWARD_SPEED * 1023) / 2048;
  final double WRIST_MM_FORWARD_KI = 0;
  final double WRIST_MM_FORWARD_KD = 0;
  final double WRIST_MM_FORWARD_KF = .07; // (.7 * 1023) / 10000  
  final int WRIST_MM_FORWARD_VELOCITY = 10000;
  final int WRIST_MM_FORWARD_ACCELERATION = (int)(WRIST_MM_FORWARD_VELOCITY / 1); // 1 second to full velocity
  final double WRIST_MM_REVERSE_KP = (WRIST_REVERSE_SPEED * 1023) / 2048;
  final double WRIST_MM_REVERSE_KI = 0;
  final double WRIST_MM_REVERSE_KD = 0;
  final double WRIST_MM_REVERSE_KF = 0.058; // (-.4 * 1023) / -7000
  final int WRIST_MM_REVERSE_VELOCITY = 7000;
  final int WRIST_MM_REVERSE_ACCELERATION = (int)(WRIST_MM_REVERSE_VELOCITY / 1); // 1 second to full velocity
  final double WRIST_HOLD_POSITION_KP = (WRIST_FORWARD_SPEED * 1023) / 512; // Tuned manually (ARM_FORWARD_SPEED * 1023) / 2048;
  final double WRIST_HOLD_POSITION_KI = 0;
  final double WRIST_HOLD_POSITION_KD = 0;
  final double WRIST_HOLD_POSITION_KF = 0;
  // Encoder Measurements for the relevant scoring positions
  final static int ARM_STOW = 0;
  final static int WRIST_STOW = 0;
  final static int ARM_GROUND_PICKUP = 40000;
  final static int WRIST_GROUND_PICKUP = 7500;
  final static int ARM_SUBSTATION = 0;
  final static int WRIST_SUBSTATION = 45000;
  final static int ARM_SCORE_LOW = 0;
  final static int WRIST_SCORE_LOW = 15000;
  final static int ARM_CONE_MID = 36500;
  final static int WRIST_CONE_MID = 49000;
  final static int ARM_CONE_HIGH = 46300;
  final static int WRIST_CONE_HIGH = 57000;
  final static int ARM_CUBE_HIGH = 46300;
  final static int WRIST_CUBE_HIGH = 52000;
  final static int ARM_CUBE_MID = 35000;
  final static int WRIST_CUBE_MID = 30000;
  final static int ARM_POSITION_TOLERANCE = 100;
  final static int WRIST_POSITION_TOLERANCE = 100;
  final int PID_PRIMARY = 0;

  /** Creates a new Arm. */
  public Arm() {
    // Configure top and bottom arm talons. NOTE: set invertType such that positive
    // input rotates to FRONT of robot
    m_arm_motor = new WPI_TalonFX(ARM_MOTOR);
    m_wrist_motor = new WPI_TalonFX(WRIST_MOTOR);

    // Factory default the talons
    m_arm_motor.configFactoryDefault();
    m_wrist_motor.configFactoryDefault();

    TalonFXConfiguration arm_config = new TalonFXConfiguration();
    TalonFXConfiguration wrist_config = new TalonFXConfiguration();

    // Setup the PID control variables for arm movement
    SlotConfiguration arm_mm = arm_config.slot0;
    arm_mm.allowableClosedloopError = 10;
    // arm_manual_forward.closedLoopPeakOutput = ARM_FORWARD_SPEED;
    arm_mm.closedLoopPeakOutput = .4;
    arm_mm.closedLoopPeriod = 1;
    arm_mm.kP = ARM_MM_KP;
    arm_mm.kI = ARM_MM_KI;
    arm_mm.kD = ARM_MM_KD;
    arm_mm.kF = ARM_MM_KF;
    arm_config.slot0 = arm_mm;

    SlotConfiguration arm_hold_config = arm_config.slot2;
    arm_hold_config.allowableClosedloopError = 10;
    arm_hold_config.closedLoopPeriod = 1;
    arm_hold_config.kP = ARM_HOLD_POSITION_KP;
    arm_hold_config.kI = ARM_HOLD_POSITION_KI;
    arm_hold_config.kD = ARM_HOLD_POSITION_KD;
    arm_hold_config.kF = ARM_HOLD_POSITION_KF;
    arm_config.slot2 = arm_hold_config;
  

    SlotConfiguration wrist_mm_forward = wrist_config.slot0;
    wrist_mm_forward.allowableClosedloopError = 10;
    wrist_mm_forward.closedLoopPeakOutput = WRIST_FORWARD_SPEED;
    wrist_mm_forward.closedLoopPeriod = 1;
    wrist_mm_forward.kP = WRIST_MM_FORWARD_KP;
    wrist_mm_forward.kI = WRIST_MM_FORWARD_KI;
    wrist_mm_forward.kD = WRIST_MM_FORWARD_KD;
    wrist_mm_forward.kF = WRIST_MM_FORWARD_KF;
    wrist_config.slot0 = wrist_mm_forward;

    SlotConfiguration wrist_mm_reverse = wrist_config.slot1;
    wrist_mm_reverse.allowableClosedloopError = 10;
    wrist_mm_reverse.closedLoopPeakOutput = Math.abs(WRIST_REVERSE_SPEED); // Must be an absolute value
    wrist_mm_reverse.closedLoopPeriod = 1;
    wrist_mm_reverse.kP = WRIST_MM_REVERSE_KP;
    wrist_mm_reverse.kI = WRIST_MM_REVERSE_KI;
    wrist_mm_reverse.kD = WRIST_MM_REVERSE_KD;
    wrist_mm_reverse.kF = WRIST_MM_REVERSE_KF;
    wrist_config.slot1 = wrist_mm_reverse;

    SlotConfiguration wrist_hold_config = wrist_config.slot2;
    wrist_hold_config.allowableClosedloopError = 10;
    wrist_hold_config.closedLoopPeriod = 1;
    wrist_hold_config.kP = WRIST_HOLD_POSITION_KP;
    wrist_hold_config.kI = WRIST_HOLD_POSITION_KI;
    wrist_hold_config.kD = WRIST_HOLD_POSITION_KD;
    wrist_hold_config.kF = WRIST_HOLD_POSITION_KF;
    wrist_config.slot2 = wrist_hold_config;

    // Setup the ARM stage motor
    m_arm_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ARM_MM_SLOT, 0);
    arm_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m_arm_motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_arm_motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // Configure limit switches
    arm_config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    arm_config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    arm_config.reverseSoftLimitEnable = false;
    arm_config.forwardSoftLimitEnable = true;
    arm_config.reverseSoftLimitThreshold = ARM_REVERSE_SOFT_LIMIT;
    arm_config.forwardSoftLimitThreshold = ARM_FORWARD_SOFT_LIMIT;

    // Set current limits for the ARM
    StatorCurrentLimitConfiguration stator_limit = arm_config.statorCurrLimit;
    stator_limit.currentLimit = ARM_MAX_STATOR_CURRENT;
    stator_limit.enable = true;
    stator_limit.triggerThresholdCurrent = ARM_MAX_STATOR_CURRENT + 5;
    arm_config.statorCurrLimit = stator_limit;

    // Set max speeds for output
    arm_config.peakOutputForward = ARM_FORWARD_SPEED;
    arm_config.peakOutputReverse = ARM_REVERSE_SPEED;

    // Configure the arm
    m_arm_motor.configAllSettings(arm_config, 10);

    // set arm inversion
    m_arm_motor.setInverted(TalonFXInvertType.Clockwise);

    // set arm brake mode
    m_arm_motor.setNeutralMode(NeutralMode.Brake);

    // Configure the Wrist motor
    m_wrist_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ARM_MM_SLOT, 0);
    wrist_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m_wrist_motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen);
    m_wrist_motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen);

    // Configure limit switches
    wrist_config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    wrist_config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    wrist_config.reverseSoftLimitEnable = true;
    wrist_config.forwardSoftLimitEnable = true;
    wrist_config.reverseSoftLimitThreshold = WRIST_REVERSE_SOFT_LIMIT;
    wrist_config.forwardSoftLimitThreshold = WRIST_FORWARD_SOFT_LIMIT;

    // Set current limits for the Wrist
    stator_limit = wrist_config.statorCurrLimit;
    stator_limit.currentLimit = WRIST_MAX_STATOR_CURRENT;
    stator_limit.enable = true;
    stator_limit.triggerThresholdCurrent = WRIST_MAX_STATOR_CURRENT + 1;
    wrist_config.statorCurrLimit = stator_limit;

    // Set max speeds for output
    wrist_config.peakOutputForward = WRIST_FORWARD_SPEED;
    wrist_config.peakOutputReverse = WRIST_REVERSE_SPEED;
    // Configure the wrist
    m_wrist_motor.configAllSettings(wrist_config, 10);
    m_wrist_motor.setInverted(TalonFXInvertType.Clockwise);
    m_wrist_motor.setNeutralMode(NeutralMode.Brake);

    // Top and bottom arm limit switches, these DIOs are for the LEDs to light up
    // when the limits are hit
    m_arm_forward_limit = new DigitalInput(ARM_FORWARD_LIMIT_SWITCH);
    m_arm_reverse_limit = new DigitalInput(ARM_REVERSE_LIMIT_SWITCH);

    m_wrist_forward_limit = new DigitalInput(WRIST_FORWARD_LIMIT_SWITCH);
    m_wrist_reverse_limit = new DigitalInput(WRIST_REVERSE_LIMIT_SWITCH);

    // Setup the network tables publishers to push data to the dashboard
    NetworkTable shuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Arm Test");
    m_arm_position = shuffleboard.getDoubleTopic("ArmEncoder").publish();
    m_wrist_position = shuffleboard.getDoubleTopic("WristEncoder").publish();

    m_wrist_motor_rev = shuffleboard.getDoubleTopic("Wrist Rev").publish();
    m_arm_motor_rev = shuffleboard.getDoubleTopic("Arm Rev").publish();
    m_arm_mm_error = shuffleboard.getDoubleTopic("Arm MM Error").publish();
    m_wrist_mm_error = shuffleboard.getDoubleTopic("Wrist MM Error").publish();
    m_super_position = shuffleboard.getStringTopic("Super Position").publish();



    m_current_position = SuperStructurePosition.Stowed;
    m_requested_position = SuperStructurePosition.Stowed;

    // puts the illegal transitions into an arraylist to use in our HashMap
    stowed_illegal_transitions.add(SuperStructurePosition.GroundPickup);
    ground_pickup_illegal_transitions.add(SuperStructurePosition.Stowed);

    // The HashMap that associates specific illegal transitions per each position
    // (as needed)
    illegal_transitions.put(SuperStructurePosition.Stowed, stowed_illegal_transitions);
    illegal_transitions.put(SuperStructurePosition.GroundPickup, ground_pickup_illegal_transitions);

    // Set the motors to hold their initial positions stowed to try and minimize
    // slop until we
    // deliberately move them.
    m_arm_motor.selectProfileSlot(ARM_HOLD_POSITION_SLOT, 0);
    m_arm_motor.set(ControlMode.Position, SuperStructurePosition.Stowed.arm_position);
    m_wrist_motor.selectProfileSlot(WRIST_HOLD_POSITION_SLOT, 0);
    m_wrist_motor.set(ControlMode.Position, SuperStructurePosition.Stowed.wrist_position);

    m_arm_state = ArmState.Holding;
  }

  public int getWristReverseLimitFromMotor(){
    return m_wrist_motor.isRevLimitSwitchClosed();
  }

  @Override
  public void periodic() {
    m_arm_position.set(m_arm_motor.getSelectedSensorPosition());
    m_wrist_position.set(m_wrist_motor.getSelectedSensorPosition());

    m_wrist_motor_rev.set(m_wrist_motor.isRevLimitSwitchClosed());
    m_arm_motor_rev.set(m_arm_motor.isRevLimitSwitchClosed());

    switch(m_arm_state){
      case Inactive:
        // If the arm isn't do anything, zero both.
        if(m_wrist_motor.isRevLimitSwitchClosed() > 0){
          m_wrist_motor.setSelectedSensorPosition(0);
        }
        if(m_arm_motor.isRevLimitSwitchClosed() > 0){
          m_arm_motor.setSelectedSensorPosition(0);
        }
        break;
      case Holding:
        // We may not be able to trust the arm's motion after a command
        // so only deal with the wrist on hold.
        if(m_wrist_motor.isRevLimitSwitchClosed() > 0){
          m_wrist_motor.setSelectedSensorPosition(0);
        }
        break;
      case Moving:
        // Don't do anything
        break;
      default:
        // Reset the wrist only by default
        if(m_wrist_motor.isRevLimitSwitchClosed() > 0){
          m_wrist_motor.setSelectedSensorPosition(0);
        }
        break;
    }


    // if(m_arm_motor.isRevLimitSwitchClosed() > 0){
    //   m_arm_motor.setSelectedSensorPosition(0);
    // }

    // checkReverseLimits();

    checkArmSuperState();

    // TODO Don't keep this in here:
    m_super_position.set(m_current_position.toString());
  }

  // ==================================== Arm State Machine Methods
  // =======================================
  public void checkArmSuperState() {
    if( m_arm_motor.getSelectedSensorPosition() <= 0 && m_wrist_motor.getSelectedSensorPosition() <= 0){
      m_current_position = SuperStructurePosition.Stowed;
    }
  }

  public void requestArmMove(SuperStructurePosition requested_position) {
    m_requested_position = requested_position;
  }

  public SuperStructurePosition getArmCurrentState() {
    return m_current_position;
  }

  public boolean isArmSuperAtRequestedPos() {
    double arm_pos = m_arm_motor.getSelectedSensorPosition();
    double wrist_pos = m_wrist_motor.getSelectedSensorPosition();
    boolean isAtPosition = false;

    double arm_err = Math.abs(m_requested_position.arm_position - arm_pos);
    double wrist_err = Math.abs(m_requested_position.wrist_position - wrist_pos);

    if (arm_err < ARM_POSITION_TOLERANCE && wrist_err < WRIST_POSITION_TOLERANCE) {
      isAtPosition = true;
    }

    return isAtPosition;
  }

  public boolean isTransitionInvalid(SuperStructurePosition requestedPosition) {
    boolean isInvalid = false;

    // If the position is a known position, we can make a smart decision
    if( m_current_position != SuperStructurePosition.Manual ){
      if( illegal_transitions.containsKey(m_current_position) ){
        ArrayList<SuperStructurePosition> illegals = illegal_transitions.get(m_current_position);

        for( SuperStructurePosition pos : illegals ){
          if (requestedPosition == pos){
            // System.out.println("Invalid request: " + m_current_position + " to " + pos);
            isInvalid = true;
            break;
          }
        }
      }
    } else {
      // If the position is Manual, we just need to make sure the wrist is in a safe zone so we don't
      // clip the bumpers on the move
      double wrist_pos = m_wrist_motor.getSelectedSensorPosition();
      if( wrist_pos <= SAFE__MOVE_WRIST_POSITION){
        // System.out.println("Invalid Manual to " + requestedPosition);
        isInvalid = true;
      }
    }
    
    // TODO Set some variable so people know it's invalid like LED or Boolean flash
    return isInvalid;
  }

  public void forceSetCurrentPos() { // FOR TESTING ONLY DO NOT USE AT COMPS
    m_current_position = m_requested_position;
  }

  public void set_current_position( SuperStructurePosition position ){
    m_current_position = position;
  }
  // ======================================================================================================

  public void moveArmManually(double bottom_speed, double top_speed) {
    m_arm_motor.set(ControlMode.PercentOutput, bottom_speed);
    m_wrist_motor.set(ControlMode.PercentOutput, top_speed);

    if (m_arm_forward_limit.get() && bottom_speed < 0) {
      m_arm_motor.set(ControlMode.PercentOutput, 0);
    }

    if (m_arm_reverse_limit.get() && top_speed < 0) {
      m_wrist_motor.set(ControlMode.PercentOutput, 0);
    }
  }

  // DO NOT USE DURING MATCHES!!! ONLY RUN IN THE PITS!!!
  public void reset_arm_pos() {
    m_arm_control_mode = ArmControlMode.Automatic;
    // while BOTH top and bottoms are false (not triggered)
    while (!m_arm_forward_limit.get() && !m_arm_reverse_limit.get()) {
      // only drive base if limit is not hit
      if (!m_arm_forward_limit.get()) {
        m_arm_motor.set(ControlMode.PercentOutput, -ARM_FORWARD_SPEED);
      } else {
        m_arm_motor.set(ControlMode.PercentOutput, 0);
      }

      // only drive upper stage if limit is not hit
      if (!m_arm_reverse_limit.get()) {
        m_wrist_motor.set(ControlMode.PercentOutput, -WRIST_FORWARD_SPEED);
      } else {
        m_wrist_motor.set(ControlMode.PercentOutput, 0);
      }
    }

    // Zero encoders once both stages are reset
    m_arm_motor.setSelectedSensorPosition(0);
    m_wrist_motor.setSelectedSensorPosition(0);
  }

  public WPI_TalonFX getBottomStageMotor() {
    return m_arm_motor;
  }

  public WPI_TalonFX getTopStageMotor() {
    return m_wrist_motor;
  }

  public ArmState getArmState() {
    return m_arm_state;
  }

  public ArmControlMode getArmControlMode() {
    return m_arm_control_mode;
  }

  public double getSuperStructureWristPosition() {

    return m_wrist_motor.getSelectedSensorPosition();
  }

  public double getSuperStructureArmPosition(){
    return m_arm_motor.getSelectedSensorPosition();
  }

  public ArmTask getArmTask() {
    return arm_task;
  }

  public DigitalInput getBottomLimitSwitch() {
    return m_arm_forward_limit;
  }

  public DigitalInput getTopLimitSwitch() {
    return m_arm_reverse_limit;
  }

  // private int radiansToNativeUnits(double radians) {
  //   double ticks = (radians / (2 * Math.PI)) * 2048;
  //   return (int) ticks;
  // }

  // Converts to native units per 100 ms
  // private int velocityToNativeUnits(double radPerSec) {
  //   double radPer100ms = radPerSec / 1000;
  //   return radiansToNativeUnits(radPer100ms);
  // }

  public boolean getArmForwardLimit() {
    return m_arm_forward_limit.get();
  }

  public boolean getArmReverseLimit() {
    return m_arm_reverse_limit.get();
  }

  public boolean getWristForwardLimit() {
    return m_wrist_forward_limit.get();
  }

  public boolean getWristReverseLimit() {
    return m_wrist_reverse_limit.get();
  }

  public void drive_manually(double arm_speed, double wrist_speed) {
    m_arm_motor.set(ControlMode.PercentOutput, arm_speed);
    m_wrist_motor.set(ControlMode.PercentOutput, wrist_speed);
  }

  public void checkReverseLimits() {
    // falcon hard limit returns 1 if closed, 0 if open. Our limits are normally
    // open
    if (m_arm_motor.isRevLimitSwitchClosed() == 1) {
      // System.out.println("Arm Switch close");
      m_arm_motor.setSelectedSensorPosition(0);
    }

    if (m_wrist_motor.isRevLimitSwitchClosed() == 1) {
      // System.out.println("Wrist Switch close");
      m_wrist_motor.setSelectedSensorPosition(0);
    }
  }

  /**
   * Convert encoder ticks to degrees for the arm
   */
  public double armToDegrees(double positionCounts) {
    return positionCounts * (360.0 / (ARM_GEAR_RATIO * 2048.0));
  }

  /**
   * 
   */
  public double degreesToFalconARM(double degrees) {
      return degrees / (360.0 / (ARM_GEAR_RATIO * 2048.0));
  }

  /**
   * Convert encoder ticks to degrees for the arm
   */
  public double wristToDegrees(double positionCounts) {
    return positionCounts * (360.0 / (WRIST_GEAR_RATIO * 2048.0));
  }

  /**
   * 
   */
  public double degreesToFalconWrist(double degrees) {
      return degrees / (360.0 / (WRIST_GEAR_RATIO * 2048.0));
  }

  public void configure_wrist_motion_magic_test(double velocity, double time_to_velo, double kP, boolean isForward){
     // Dividing by zero is very bad, will crash most systems. 
		if( time_to_velo == 0 ){
			time_to_velo = 1;
		}
		double acceleration = velocity / time_to_velo;

    if (isForward ){
      m_wrist_motor.selectProfileSlot(WRIST_MM_FORWARD_SLOT, 0);
      m_wrist_motor.config_kP(WRIST_MM_FORWARD_SLOT, kP);
      m_arm_motor.config_kF(WRIST_MM_FORWARD_SLOT, (WRIST_FORWARD_SPEED * 1023)/velocity);
      m_wrist_motor.configMotionCruiseVelocity(velocity);
      m_wrist_motor.configMotionAcceleration(acceleration);
    } else {
      m_wrist_motor.selectProfileSlot(WRIST_MM_REVERSE_SLOT, 0);
      m_wrist_motor.config_kP(WRIST_MM_REVERSE_SLOT, kP);
      m_arm_motor.config_kF(WRIST_MM_REVERSE_SLOT, (WRIST_REVERSE_SPEED * 1023)/velocity);
      m_wrist_motor.configMotionCruiseVelocity(velocity);
      m_wrist_motor.configMotionAcceleration(acceleration);
    }
    

    // If we're within 10 ticks we feel good
    // m_wrist_motor.configAllowableClosedloopError(ARM_MM_SLOT, 10);
  }

  public void move_wrist_motion_magic(int target_in_ticks, boolean isForward){
    if( isForward ){
      m_wrist_motor.set(ControlMode.MotionMagic, target_in_ticks, DemandType.ArbitraryFeedForward, getWristArbFF());
    }else {
      m_wrist_motor.set(ControlMode.MotionMagic, target_in_ticks);
    }
    // m_wrist_motor.set(ControlMode.MotionMagic, target_in_ticks);
  }
  
  private double getWristArbFF(){
    return WRIST_FORWARD_COSINE_FF * Math.cos(wristToDegrees(m_wrist_motor.getSelectedSensorPosition()));
  }

  public boolean is_wrist_mm_done(int target_ticks){
    double wrist_pos = m_wrist_motor.getSelectedSensorPosition();

    m_wrist_mm_error.set(Math.abs(target_ticks - wrist_pos));
    return Math.abs((target_ticks - wrist_pos)) < WRIST_POSITION_TOLERANCE;
  }

  public void configure_arm_motion_magic_test(double velocity, double time_to_velo, double kP){
    // Dividing by zero is very bad, will crash most systems. 
		if( time_to_velo == 0 ){
			time_to_velo = 1;
		}
		double acceleration = velocity / time_to_velo;

    m_arm_motor.selectProfileSlot(ARM_MM_SLOT, 0);
    m_arm_motor.config_kP(ARM_MM_SLOT, kP);
    m_arm_motor.config_kF(ARM_MM_SLOT, (ARM_FORWARD_SPEED * 1023)/velocity);
    m_arm_motor.configMotionCruiseVelocity(velocity);
    m_arm_motor.configMotionAcceleration(acceleration);
    // If we're within 10 ticks we feel good
    // m_arm_motor.configAllowableClosedloopError(ARM_MM_SLOT, 10);
  }

  public void move_arm_motion_magic(int target_in_ticks){
    m_arm_motor.set(ControlMode.MotionMagic, target_in_ticks);
  }

  public boolean is_arm_mm_done(int target_ticks){
    double arm_pos = m_arm_motor.getSelectedSensorPosition();
    m_arm_mm_error.set(Math.abs(target_ticks - arm_pos));
    return Math.abs((target_ticks - arm_pos)) < ARM_POSITION_TOLERANCE;
  }

  public void set_arm_encoder_to_zero() {
    m_arm_motor.setSelectedSensorPosition(0);
  }

  public boolean is_arm_rev_limit_hit() {
    return m_arm_motor.isRevLimitSwitchClosed() == 1;
  }

  public boolean is_arm_fwd_limit_hit() {
    return m_arm_motor.isFwdLimitSwitchClosed() == 1;
  }

  public boolean is_wrist_rev_limit_hit() {
    return m_wrist_motor.isRevLimitSwitchClosed() == 1;
  }

  public boolean is_wrist_fwd_limit_hit() {
    return m_wrist_motor.isFwdLimitSwitchClosed() == 1;
  }

  public void holdPosition(){
    double arm_pos = m_arm_motor.getSelectedSensorPosition();
    double wrist_pos = m_wrist_motor.getSelectedSensorPosition();

    // Set the arm and wrist to their hold position slots as primary pids
    m_arm_motor.selectProfileSlot(ARM_HOLD_POSITION_SLOT, 0);
    m_wrist_motor.selectProfileSlot(WRIST_HOLD_POSITION_SLOT, 0);

    // Set the motor to hold with PID
    m_arm_motor.set(ControlMode.Position, arm_pos);
    m_wrist_motor.set(ControlMode.Position, wrist_pos, DemandType.ArbitraryFeedForward, getWristArbFF());
  }

  public void set_current_position_to_manual() {
    m_current_position = SuperStructurePosition.Manual;
  }

  public void configure_wrist_motion_magic(int target_ticks, boolean isForward){
    if( isForward ){
      m_wrist_motor.selectProfileSlot(WRIST_MM_FORWARD_SLOT, PID_PRIMARY);
    }
    else {
      m_wrist_motor.selectProfileSlot(WRIST_MM_REVERSE_SLOT, PID_PRIMARY);
    }
  }

  public boolean isBackwardMovement(SuperStructurePosition pos){
    double wristPos = m_wrist_motor.getSelectedSensorPosition();
    double armPos = m_arm_motor.getSelectedSensorPosition();
    System.out.println("Moving arm Backward: " + (!(wristPos < pos.wrist_position) || !(armPos < pos.arm_position)));
    return  !(wristPos < pos.wrist_position) || !(armPos < pos.arm_position);
  }

  public void configure_arm_motion_magic(){
    m_arm_motor.selectProfileSlot(ARM_MM_SLOT, PID_PRIMARY);
  }
  ////////////////////// ARM INLINE COMMANDS /////////////////////


  public CommandBase setArmEncoderToZero() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_arm_motor.setSelectedSensorPosition(0);
        });
  }

  public CommandBase setWristEncoderToZero() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_wrist_motor.setSelectedSensorPosition(0);
        });
  }

  public CommandBase setArmMaxSpeed() {
    return runOnce(
        () -> {
          NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard")
              .getSubTable("Arm Test");
          double forward_speed = driveTab.getEntry("Arm Fwd Spd").getDouble(ARM_FORWARD_SPEED);
          double reverse_speed = driveTab.getEntry("Arm Rev Spd").getDouble(ARM_REVERSE_SPEED);

          // Just in case they put values that aren't positive or negative on Shuffleboard
          // as an accident, make sure it's right
          if (reverse_speed > 0) {
            reverse_speed = reverse_speed * -1;
          }

          if (forward_speed < 0) {
            forward_speed = forward_speed * -1;
          }

          m_arm_motor.configPeakOutputForward(forward_speed);
          m_arm_motor.configPeakOutputReverse(reverse_speed);
        });
  }

  public CommandBase setWristMaxSpeed() {
    return runOnce(
        () -> {
          NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard")
              .getSubTable("Arm Test");
          double forward_speed = driveTab.getEntry("Wrist Fwd Spd").getDouble(WRIST_FORWARD_SPEED);
          double reverse_speed = driveTab.getEntry("Wrist Rev Spd").getDouble(WRIST_REVERSE_SPEED);

          // Just in case they put values that aren't positive or negative on Shuffleboard
          // as an accident, make sure it's right
          if (reverse_speed > 0) {
            reverse_speed = reverse_speed * -1;
          }

          if (forward_speed < 0) {
            forward_speed = forward_speed * -1;
          }

          m_wrist_motor.configPeakOutputForward(forward_speed);
          m_wrist_motor.configPeakOutputReverse(reverse_speed);
        });
  }

  public CommandBase stopArmAndWristCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_arm_motor.set(ControlMode.PercentOutput, 0);
          m_wrist_motor.set(ControlMode.PercentOutput, 0);
        });
  }

  public CommandBase requestMoveSuperstructure(SuperStructurePosition position){
    return runOnce(
      () -> {
        System.out.println("Moving Superposition to: " + position.toString());
        m_current_position = position;
        // m_arm_motor.set(ControlMode.MotionMagic, position.arm_position);
        // m_wrist_motor.set(ControlMode.MotionMagic, position.wrist_position);
      });
  }

  public CommandBase configureWristMM(BooleanSupplier isForward){
    return runOnce(
      () -> {
        /* If we're going in the forward direciton we need to select a specific 
         * PID slot on the controller, versus backward being different. 
         * 
         * Then just set the MM target once the configuration is done
         */
        if(isForward.getAsBoolean()){
          m_wrist_motor.selectProfileSlot(WRIST_MM_FORWARD_SLOT, PID_PRIMARY);
        }
        else {
          m_wrist_motor.selectProfileSlot(WRIST_MM_REVERSE_SLOT, PID_PRIMARY);
        }
      });
  }
  public CommandBase moveWristToPositionMM(SuperStructurePosition position, BooleanSupplier isForward){
    return run(
      () -> {
        if( isForward.getAsBoolean() ){
          m_wrist_motor.set(ControlMode.MotionMagic, position.wrist_position, DemandType.ArbitraryFeedForward, getWristArbFF());
        } else {
          m_wrist_motor.set(ControlMode.MotionMagic, position.wrist_position);
        }
      });
  }
}