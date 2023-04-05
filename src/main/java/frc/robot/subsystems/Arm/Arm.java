// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Wrist.Wrist;

public class Arm extends SubsystemBase {
  public WPI_TalonFX m_arm_motor;
  DigitalInput m_arm_forward_limit;
  DigitalInput m_arm_reverse_limit;
  DoublePublisher m_arm_position, m_wrist_position, m_arm_motor_rev, m_arm_mm_error;
  StringPublisher m_super_position;

  private ArmState m_arm_state;
  private ArmControlMode m_arm_control_mode;
  private ArmSuperStructurePosition m_current_position, m_requested_position;
  private ArmTask arm_task;
  private HashMap<ArmSuperStructurePosition, ArrayList<ArmSuperStructurePosition>> illegal_transitions = new HashMap<ArmSuperStructurePosition, ArrayList<ArmSuperStructurePosition>>();
  private ArrayList<ArmSuperStructurePosition> stowed_illegal_transitions = new ArrayList<ArmSuperStructurePosition>();
  // private ArrayList<SuperStructurePosition> ground_pickup_illegal_transitions = new ArrayList<SuperStructurePosition>();

  public static enum ArmState {
    Moving,
    Holding,
    Inactive,
    Stowed
  }

  public enum ArmControlMode { // Mostly for debugging information, could be used as an arm safety measure in
                               // emergencies
    Manual,
    Automatic,
  }

  public enum ArmSuperStructurePosition {
    Stowed(ARM_STOW){
      @Override
      public String toString() {
          return "Stowed";
      }
    },
    GroundPickup(ARM_GROUND_PICKUP){
      @Override
      public String toString() {
          return "Ground Pickup";
      }
    },
    SubstationPickup(ARM_SUBSTATION){
      @Override
      public String toString() {
          return "Substation";
      }
    },
    ScoreLow(ARM_SCORE_LOW){
      @Override
      public String toString() {
          return "Score Low";
      }},
    ScoreConeMid(ARM_CONE_MID){
      @Override
      public String toString() {
          return "Cone Mid";
      }
    },
    ScoreConeHigh(ARM_CONE_HIGH){
      @Override
      public String toString() {
          return "Cone High";
      }
    },
    ScoreCubeMid(ARM_CUBE_MID){
      @Override
      public String toString() {
          return "Cube Mid";
      }
    },
    ScoreCubeHigh(ARM_CUBE_HIGH){
      @Override
      public String toString() {
          return "Cube High";
      }
    },
    Manual(1){
      @Override
      public String toString() {
          return "Manual";
      }
    }; // Default small values to make sure calculations won't fail

    private int arm_position;
    private ArrayList<ArmSuperStructurePosition> illegal_transitions;

    private ArmSuperStructurePosition(int arm_pos) {
      arm_position = arm_pos;

      illegal_transitions = new ArrayList<ArmSuperStructurePosition>();
    }

    private ArmSuperStructurePosition(int arm_pos, ArmSuperStructurePosition illegalPosition) {
      arm_position = arm_pos;

      illegal_transitions = new ArrayList<ArmSuperStructurePosition>();
      illegal_transitions.add(illegalPosition);
    }

    public int getArmPosition(){
      return this.arm_position;
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
  final double ARM_FORWARD_SPEED = .2;
  final double ARM_REVERSE_SPEED = -.2;
 
  final double ARM_GEAR_RATIO = 4 * 4 * 4 * (2/1);
  final int ARM_REVERSE_SOFT_LIMIT = 0;
  final int ARM_FORWARD_SOFT_LIMIT = 93000;
  final int SAFE__MOVE_WRIST_POSITION = 3000; // Puts the wrist up at 11 degrees
  // final int ARM_FORWARD_SOFT_LIMIT = (int)(2048 * ARM_GEAR_RATIO * 1/6); // 60
  // degrees rotation
  final double ARM_MAX_STATOR_CURRENT = 40;
  final double ARM_STATOR_CURRENT_TRIGGER = 100;
  final int ARM_MM_SLOT = 0;
  final int ARM_HOLD_POSITION_SLOT = 2;



  // The wrist travels 90 degrees total, for manual steps try to go 3 degrees at a
  // time
  // The arm travels 46 degrees total, for manual steps try one degree at a time
  final int ARM_POSITION_STEP = (int) (ARM_FORWARD_SOFT_LIMIT / 23);

  /*
   * KP Calculations are:
   * Desired Percent * full throttle / error
   * That means, we want to saturate the feedback (full recovery) with an error
   * value
   */
  final double ARM_MM_KP = 2.1; // Tuned manually (ARM_FORWARD_SPEED * 1023) / 2048;
  final double ARM_MM_KI = 0;
  final double ARM_MM_KD = 0;
  final double ARM_MM_KF = 0.43; // (.4 * 1023) / 8000
  final double ARM_MM_FF = 0;
  final int ARM_MM_VELOCITY = 2000;
  final int ARM_MM_ACCELERATION = 700; // 1 Second to full velocity
  final double ARM_HOLD_POSITION_KP = 3.069;
  final double ARM_HOLD_POSITION_KI = 0;
  final double ARM_HOLD_POSITION_KD = 15;
  final double ARM_HOLD_POSITION_KF = 0;
  // Encoder Measurements for the relevant scoring positions
  final static int ARM_STOW = 0;
 // final static int WRIST_STOW = 0;
  final static int ARM_GROUND_PICKUP = 8000;
  //final static int WRIST_GROUND_PICKUP = 0;
  final static int ARM_SUBSTATION = 0;
 // final static int WRIST_SUBSTATION = 28500;
  final static int ARM_SCORE_LOW = 0;
 // final static int WRIST_SCORE_LOW = 4000;
  final static int ARM_CONE_MID =  9000;
 // final static int WRIST_CONE_MID = 27500;
  final static int ARM_CONE_HIGH = 25000;
 // final static int WRIST_CONE_HIGH = 0;
  final static int ARM_CUBE_HIGH = 9900;
 // final static int WRIST_CUBE_HIGH = 32000;
  final static int ARM_CUBE_MID = 0;
 // final static int WRIST_CUBE_MID = 23500;
  final static int ARM_POSITION_TOLERANCE = 250;
 // final static int WRIST_POSITION_TOLERANCE = 250;
  final int PID_PRIMARY = 0;

  /** Creates a new Arm. */
  public Arm() {
    // Configure top and bottom arm talons. NOTE: set invertType such that positive
    // input rotates to FRONT of robot
    m_arm_motor = new WPI_TalonFX(ARM_MOTOR);

    // Factory default the talons
    m_arm_motor.configFactoryDefault();

    TalonFXConfiguration arm_config = new TalonFXConfiguration();
    TalonFXConfiguration wrist_config = new TalonFXConfiguration();

    // Setup the PID control variables for arm movement
    SlotConfiguration arm_mm = arm_config.slot0;
    arm_mm.allowableClosedloopError = 10;
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
    arm_config.clearPositionOnLimitR = true;

    // // Set current limits for the ARM
    // StatorCurrentLimitConfiguration stator_limit = arm_config.statorCurrLimit;
    // stator_limit.currentLimit = ARM_MAX_STATOR_CURRENT;
    // stator_limit.enable = true;
    // stator_limit.triggerThresholdCurrent = ARM_MAX_STATOR_CURRENT + 5;
    // stator_limit.triggerThresholdTime = .1;
    // arm_config.statorCurrLimit = stator_limit;

    // Set max speeds for output
    arm_config.peakOutputForward = .4;
    arm_config.peakOutputReverse = -.4;

    // Configure the arm
    m_arm_motor.configAllSettings(arm_config, 10);

    // set arm inversion
    m_arm_motor.setInverted(TalonFXInvertType.Clockwise);

    // set arm brake mode
    m_arm_motor.setNeutralMode(NeutralMode.Brake);

    // Configure the Wrist motor

    // // Set current limits for the Wrist
    // stator_limit = wrist_config.statorCurrLimit;
    // stator_limit.currentLimit = WRIST_MAX_STATOR_CURRENT;
    // stator_limit.enable = true;
    // stator_limit.triggerThresholdCurrent = WRIST_STATOR_CURRENT_TRIGGER;
    // stator_limit.triggerThresholdTime = .001;
    // wrist_config.statorCurrLimit = stator_limit;

    // Top and bottom arm limit switches, these DIOs are for the LEDs to light up
    // when the limits are hit
    m_arm_forward_limit = new DigitalInput(ARM_FORWARD_LIMIT_SWITCH);
    m_arm_reverse_limit = new DigitalInput(ARM_REVERSE_LIMIT_SWITCH);


    // Setup the network tables publishers to push data to the dashboard
    NetworkTable shuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Arm Test");
    m_arm_position = shuffleboard.getDoubleTopic("ArmEncoder").publish();

    m_arm_mm_error = shuffleboard.getDoubleTopic("Arm Error").publish();
    m_super_position = shuffleboard.getStringTopic("Super Position").publish();



    m_current_position = ArmSuperStructurePosition.Stowed;
    m_requested_position = ArmSuperStructurePosition.Stowed;

    // puts the illegal transitions into an arraylist to use in our HashMap
    stowed_illegal_transitions.add(ArmSuperStructurePosition.GroundPickup);
    // ground_pickup_illegal_transitions.add(SuperStructurePosition.Stowed);

    // The HashMap that associates specific illegal transitions per each position
    // (as needed)
    illegal_transitions.put(ArmSuperStructurePosition.Stowed, stowed_illegal_transitions);
    // illegal_transitions.put(SuperStructurePosition.GroundPickup, ground_pickup_illegal_transitions);

    // Set the motors to hold their initial positions stowed to try and minimize
    // slop until we
    // deliberately move them.
    m_arm_motor.selectProfileSlot(ARM_HOLD_POSITION_SLOT, 0);
    m_arm_motor.set(ControlMode.Position, ArmSuperStructurePosition.Stowed.arm_position);


    m_arm_state = ArmState.Holding;
  }





  @Override
  public void periodic() {
    m_arm_position.set(m_arm_motor.getSelectedSensorPosition());

    switch(m_arm_state){
      case Inactive:

      case Stowed:
        // If the arm isn't doing anything, zero it.
        if(m_arm_motor.isRevLimitSwitchClosed() > 0){
          m_arm_motor.setSelectedSensorPosition(0);
        }
        break;
      case Holding:
      case Moving:
    }


    // if(m_arm_motor.isRevLimitSwitchClosed() > 0){
    //   m_arm_motor.setSelectedSensorPosition(0);
    // }

    // checkReverseLimits();

    checkArmSuperState();
    
    m_super_position.set(m_current_position.toString());


  }

  // ==================================== Arm State Machine Methods
  // =======================================
  public void checkArmSuperState() {
    if( m_arm_motor.getSelectedSensorPosition() <= 500){
      m_current_position = ArmSuperStructurePosition.Stowed;
    }
  }

  public void requestArmMove(ArmSuperStructurePosition requested_position) {
    m_requested_position = requested_position;
  }

  public ArmSuperStructurePosition getArmCurrentState() {
    return m_current_position;
  }

  public boolean isArmSuperAtRequestedPos() {
    double arm_pos = m_arm_motor.getSelectedSensorPosition();
    boolean isAtPosition = false;

    double arm_err = Math.abs(m_requested_position.arm_position - arm_pos);

    if (arm_err < ARM_POSITION_TOLERANCE) {
      isAtPosition = true;
    }

    return isAtPosition;
  }

  public boolean isTransitionInvalid(ArmSuperStructurePosition requestedPosition) {
    boolean isInvalid = false;

    // If the position is a known position, we can make a smart decision
    if( m_current_position != ArmSuperStructurePosition.Manual ){
      if( illegal_transitions.containsKey(m_current_position) ){
        ArrayList<ArmSuperStructurePosition> illegals = illegal_transitions.get(m_current_position);

        for( ArmSuperStructurePosition pos : illegals ){
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

    }
    
    // TODO Set some variable so people know it's invalid like LED or Boolean flash
    return isInvalid;
  }

  public void forceSetCurrentPos() { // FOR TESTING ONLY DO NOT USE AT COMPS
    m_current_position = m_requested_position;
  }

  public void set_current_position( ArmSuperStructurePosition position ){
    m_current_position = position;
  }
  // ======================================================================================================

  public void moveArmManually(double bottom_speed, double top_speed) {
    m_arm_motor.set(ControlMode.PercentOutput, bottom_speed);

    if (m_arm_forward_limit.get() && bottom_speed < 0) {
      m_arm_motor.set(ControlMode.PercentOutput, 0);
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

    }

    // Zero encoders once both stages are reset
    m_arm_motor.setSelectedSensorPosition(0);
  }

  public WPI_TalonFX getBottomStageMotor() {
    return m_arm_motor;
  }


  public ArmState getArmState() {
    return m_arm_state;
  }

  public ArmControlMode getArmControlMode() {
    return m_arm_control_mode;
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

  public void drive_manually(double arm_speed) {
    // m_arm_state = ArmState.Moving;
    arm_speed = MathUtil.applyDeadband(arm_speed, .01);

    arm_speed = MathUtil.clamp(arm_speed, ARM_REVERSE_SPEED, ARM_FORWARD_SPEED);

    // System.out.println("Arm Speed: " + arm_speed);
    m_arm_motor.set(ControlMode.PercentOutput, arm_speed);
  }

  public void checkArmReverseLimits() {
    // falcon hard limit returns 1 if closed, 0 if open. Our limits are normally
    // open
    if (m_arm_motor.isRevLimitSwitchClosed() == 1) {
      // System.out.println("Arm Switch close");
      m_arm_motor.setSelectedSensorPosition(0);
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

    // If we're within 10 ticks we feel good
    // m_wrist_motor.configAllowableClosedloopError(ARM_MM_SLOT, 10);
  

 

  public void configure_arm_motion_magic_test(double velocity, double time_to_velo, double kP, double kF){
    // Dividing by zero is very bad, will crash most systems. 
		if( time_to_velo == 0 ){
			time_to_velo = 1;
		}
		double acceleration = velocity / time_to_velo;

    m_arm_motor.selectProfileSlot(ARM_MM_SLOT, 0);
    m_arm_motor.config_kP(ARM_MM_SLOT, kP);
    m_arm_motor.config_kF(ARM_MM_SLOT, kF);
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

  public void holdArmPosition(){
    double arm_pos = m_arm_motor.getSelectedSensorPosition();

    // Set the arm and wrist to their hold position slots as primary pids
    m_arm_motor.selectProfileSlot(ARM_HOLD_POSITION_SLOT, 0);

    // Set the motor to hold with PID
    m_arm_motor.set(ControlMode.Position, arm_pos);
  }

  public void set_current_position_to_manual() {
    m_current_position = ArmSuperStructurePosition.Manual;
  }


  public boolean isBackwardMovement(ArmSuperStructurePosition pos){
    double armPos = m_arm_motor.getSelectedSensorPosition();
    return  !(armPos < pos.arm_position);
  }

  public void configure_arm_motion_magic(){
    m_arm_motor.selectProfileSlot(ARM_MM_SLOT, PID_PRIMARY);
    m_arm_motor.configMotionCruiseVelocity(ARM_MM_VELOCITY);
    m_arm_motor.configMotionAcceleration(ARM_MM_ACCELERATION);
  }

  public void set_state_to_inactive(){
    m_arm_state = ArmState.Inactive;
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

  public CommandBase requestMoveSuperstructure(ArmSuperStructurePosition position){
    return runOnce(
      () -> {
        System.out.println("Moving Superposition to: " + position.toString());
        m_current_position = position;
        // m_arm_motor.set(ControlMode.MotionMagic, position.arm_position);
        // m_wrist_motor.set(ControlMode.MotionMagic, position.wrist_position);
      });
  }

 

  public CommandBase set_state(ArmState state){
    return runOnce(
      () -> {
        m_arm_state = state;
      });
  }

  public boolean is_arm_deployed(){
    return m_arm_motor.getSelectedSensorPosition() >= 25000;
  }
  public CommandBase deployArm(){
    return run(
      () -> {
        m_arm_motor.set(ControlMode.PercentOutput, ARM_FORWARD_SPEED);
      }
    ).until(this::is_arm_deployed);
  }

}