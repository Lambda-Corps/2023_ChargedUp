// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Wrist;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.SuperStructurePosition;

public class Wrist extends SubsystemBase {

  public WPI_TalonFX  m_wrist_motor;
  DigitalInput  m_wrist_reverse_limit, m_wrist_forward_limit;
  DigitalInput m_arm_forward_limit;
  DigitalInput m_arm_reverse_limit;
  DoublePublisher  m_wrist_position, m_wrist_motor_rev, m_wrist_mm_error;
  StringPublisher m_super_position;

  private WristState m_wrist_state;
  private WristControlMode m_wrist_control_mode;
  private WristTask wrist_task;
  private SuperStructurePosition m_current_position, m_requested_position;
  private HashMap<SuperStructurePosition, ArrayList<SuperStructurePosition>> m_illegal_transitions = new HashMap<SuperStructurePosition, ArrayList<SuperStructurePosition>>();
  private ArrayList<SuperStructurePosition> stowed_illegal_transitions = new ArrayList<SuperStructurePosition>();


  public static enum WristState {
    Moving,
    Holding,
    Inactive,
    Stowed
  }

  public enum WristControlMode { // Mostly for debugging information, could be used as an arm safety measure in
    // emergencies
Manual,
Automatic,
}

  public enum SuperStructurePosition {
    Stowed(WRIST_STOW){
      @Override
      public String toString() {
          return "Stowed";
      }
    },
    GroundPickup(WRIST_GROUND_PICKUP){
      @Override
      public String toString() {
          return "Ground Pickup";
      }
    },
    SubstationPickup(WRIST_SUBSTATION){
      @Override
      public String toString() {
          return "Substation";
      }
    },
    ScoreLow(WRIST_SCORE_LOW){
      @Override
      public String toString() {
          return "Score Low";
      }},
    ScoreConeMid(WRIST_CONE_MID){
      @Override
      public String toString() {
          return "Cone Mid";
      }
    },
    ScoreConeHigh(WRIST_CONE_HIGH){
      @Override
      public String toString() {
          return "Cone High";
      }
    },
    ScoreCubeMid(WRIST_CUBE_MID){
      @Override
      public String toString() {
          return "Cube Mid";
      }
    },
    ScoreCubeHigh(WRIST_CUBE_HIGH){
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

  private int wrist_position;
  private ArrayList<SuperStructurePosition> m_illegal_transitions;

  private SuperStructurePosition( int wrist_pos ) {
    wrist_position = wrist_pos;

    m_illegal_transitions = new ArrayList<SuperStructurePosition>();
  }

  private SuperStructurePosition( int wrist_pos, SuperStructurePosition illegalPosition) {
    wrist_position = wrist_pos;

    m_illegal_transitions = new ArrayList<SuperStructurePosition>();
    m_illegal_transitions.add(illegalPosition);
  }

  public int getWristPosition(){
    return this.wrist_position;
  }

  }
  

 
  public enum WristTask {
    HoldPosition,
    MoveToPosition,
    MoveManually,
    Stop,
  }
      
////////////// Constants \\\\\\\\\\\\\\\
  final double WRIST_FORWARD_SPEED = .3;
  final double WRIST_REVERSE_SPEED = -.15;
  final double WRIST_FORWARD_COSINE_FF = .15; // When arm is horizontal, calculation should be 1 * .07
  // We're trying to calculate a feed forward based on the cosine of the wrist angle (when wrist is horizontal,
  // at 90 degrees, the cosine should return 1.  Our wrist starts offset at 21 degrees relative to vertical, so
  // we want the resulting calculation to be cos(69) = 1
  final double WRIST_GEAR_RATIO = 7 * 4 * (64/24);
  final int WRIST_REVERSE_SOFT_LIMIT = -500;
  final int WRIST_FORWARD_SOFT_LIMIT = 43000;
  final double WRIST_MAX_STATOR_CURRENT = 50;
  final double WRIST_STATOR_CURRENT_TRIGGER = 100;
  final int WRIST_MM_FORWARD_SLOT = 0;
  final int WRIST_MM_REVERSE_SLOT = 1;
  final int WRIST_HOLD_POSITION_SLOT = 2;

  final int WRIST_POSITION_STEP = (int) (WRIST_FORWARD_SOFT_LIMIT / 30);


  final double WRIST_COSINE_STARTING_OFFSET = 21;
  final double WRIST_MM_FORWARD_KP = 1.6;
  final double WRIST_MM_FORWARD_KI = 0;
  final double WRIST_MM_FORWARD_KD = 0;
  final double WRIST_MM_FORWARD_KF = .17;// tuned manually
  final int WRIST_MM_FORWARD_VELOCITY = 10000;
  final int WRIST_MM_FORWARD_ACCELERATION = 10000; // 1 second to full velocity
  final double WRIST_MM_REVERSE_KP = .075;
  final double WRIST_MM_REVERSE_KI = 0;
  final double WRIST_MM_REVERSE_KD = 0;
  final double WRIST_MM_REVERSE_KF = 0.058; // (-.4 * 1023) / -7000
  final int WRIST_MM_REVERSE_VELOCITY = 6000;
  final int WRIST_MM_REVERSE_ACCELERATION = (int)(WRIST_MM_REVERSE_VELOCITY / 1); // 1 second to full velocity
  final double WRIST_HOLD_POSITION_KP = (WRIST_FORWARD_SPEED * 1023) / 512; // Tuned manually (ARM_FORWARD_SPEED * 1023) / 2048;
  final double WRIST_HOLD_POSITION_KI = 0;
  final double WRIST_HOLD_POSITION_KD = 0;
  final double WRIST_HOLD_POSITION_KF = 0;

  final int SAFE__MOVE_WRIST_POSITION = 3000; // Puts the wrist up at 11 degrees

  final static int WRIST_STOW = 0;
  final static int WRIST_GROUND_PICKUP = 0;
  final static int WRIST_SUBSTATION = 28500;
  final static int WRIST_SCORE_LOW = 4000;
  final static int WRIST_CONE_MID = 27500;
  final static int WRIST_CONE_HIGH = 0;
  final static int WRIST_CUBE_HIGH = 32000;
  final static int WRIST_CUBE_MID = 23500;
  final static int WRIST_POSITION_TOLERANCE = 250;
  final int PID_PRIMARY = 0;


  /** Creates a new Wrist */
  public Wrist() {
    m_wrist_motor = new WPI_TalonFX(WRIST_MOTOR);
    m_wrist_motor.configFactoryDefault();
    TalonFXConfiguration wrist_config = new TalonFXConfiguration();

    SlotConfiguration wrist_mm_forward = wrist_config.slot0;
    wrist_mm_forward.allowableClosedloopError = 10;
    wrist_mm_forward.closedLoopPeakOutput = .3;
    wrist_mm_forward.closedLoopPeriod = 1;
    wrist_mm_forward.kP = WRIST_MM_FORWARD_KP;
    wrist_mm_forward.kI = WRIST_MM_FORWARD_KI;
    wrist_mm_forward.kD = WRIST_MM_FORWARD_KD;
    wrist_mm_forward.kF = WRIST_MM_FORWARD_KF;
    wrist_config.slot0 = wrist_mm_forward;

    SlotConfiguration wrist_mm_reverse = wrist_config.slot1;
    wrist_mm_reverse.allowableClosedloopError = 10;
    wrist_mm_reverse.closedLoopPeakOutput = .15;
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

    // Configure the Wrist motor
    m_wrist_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,  0);
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
    wrist_config.clearPositionOnLimitR = true;

    // // Set current limits for the Wrist
    // stator_limit = wrist_config.statorCurrLimit;
    // stator_limit.currentLimit = WRIST_MAX_STATOR_CURRENT;
    // stator_limit.enable = true;
    // stator_limit.triggerThresholdCurrent = WRIST_STATOR_CURRENT_TRIGGER;
    // stator_limit.triggerThresholdTime = .001;
    // wrist_config.statorCurrLimit = stator_limit;

    // Set max speeds for output
    wrist_config.peakOutputForward = .3;
    wrist_config.peakOutputReverse = -.15;
    wrist_config.openloopRamp = .4;
    wrist_config.closedloopRamp = .3;
    // Configure the wrist
    m_wrist_motor.configAllSettings(wrist_config, 10);
    m_wrist_motor.setInverted(TalonFXInvertType.Clockwise);
    m_wrist_motor.setNeutralMode(NeutralMode.Brake);

    m_wrist_forward_limit = new DigitalInput(WRIST_FORWARD_LIMIT_SWITCH);
    m_wrist_reverse_limit = new DigitalInput(WRIST_REVERSE_LIMIT_SWITCH);

    NetworkTable shuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Arm Test");
    m_wrist_position = shuffleboard.getDoubleTopic("WristEncoder").publish();
    m_wrist_mm_error = shuffleboard.getDoubleTopic("Wrist Error").publish();

    m_current_position = SuperStructurePosition.Stowed;
    m_requested_position = SuperStructurePosition.Stowed;

    stowed_illegal_transitions.add(SuperStructurePosition.GroundPickup);
    // ground_pickup_illegal_transitions.add(SuperStructurePosition.Stowed);

    // The HashMap that associates specific illegal transitions per each position
    // (as needed)
    m_illegal_transitions.put(SuperStructurePosition.Stowed, stowed_illegal_transitions);
    // illegal_transitions.put(SuperStructurePosition.GroundPickup, ground_pickup_illegal_transitions);

    m_wrist_motor.selectProfileSlot(WRIST_HOLD_POSITION_SLOT, 0);
    m_wrist_motor.set(ControlMode.Position, SuperStructurePosition.Stowed.wrist_position);

  }

  public int getWristReverseLimitFromMotor(){
    return m_wrist_motor.isRevLimitSwitchClosed();
  }

  public TalonFX getWristMotor(TalonFX m_m_wrist_motor){
    m_m_wrist_motor = m_wrist_motor;
    return m_m_wrist_motor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_wrist_position.set(m_wrist_motor.getSelectedSensorPosition());

    switch(m_wrist_state){
      case Inactive:
      case Stowed:
        // If the arm isn't do anything, zero both.
        if(m_wrist_motor.isRevLimitSwitchClosed() > 0){
          m_wrist_motor.setSelectedSensorPosition(0);
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
    
    m_super_position.set(m_current_position.toString());

  }

  public boolean isTransitionInvalid(SuperStructurePosition requestedPosition) {
    boolean isInvalid = false;

    // If the position is a known position, we can make a smart decision
    if( m_current_position != SuperStructurePosition.Manual ){
      if( m_illegal_transitions.containsKey(m_current_position) ){
        ArrayList<SuperStructurePosition> illegals = m_illegal_transitions.get(m_current_position);

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

  public void wrist_set_current_position( SuperStructurePosition position ){
    m_current_position = position;
  }

  public void moveWristManually(double bottom_speed, double top_speed) {
    m_wrist_motor.set(ControlMode.PercentOutput, top_speed);

    if (m_arm_reverse_limit.get() && top_speed < 0) {
      m_wrist_motor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void wrist_reset_arm_pos() {
    m_wrist_control_mode = WristControlMode.Automatic;
    // while BOTH top and bottoms are false (not triggered)
    while (!m_arm_forward_limit.get() && !m_arm_reverse_limit.get()) {
      // only drive upper stage if limit is not hit
      if (!m_arm_reverse_limit.get()) {
        m_wrist_motor.set(ControlMode.PercentOutput, -WRIST_FORWARD_SPEED);
      } else {
        m_wrist_motor.set(ControlMode.PercentOutput, 0);
      }
    }

    // Zero encoders once both stages are reset
    m_wrist_motor.setSelectedSensorPosition(0);
  }

  public WPI_TalonFX getTopStageMotor() {
    return m_wrist_motor;
  }

  public double getSuperStructureWristPosition() {

    return m_wrist_motor.getSelectedSensorPosition();
  }

  public boolean getWristForwardLimit() {
    return m_wrist_forward_limit.get();
  }

  public boolean getWristReverseLimit() {
    return m_wrist_reverse_limit.get();
  }

  public WristControlMode getWristControlMode() {
    return m_wrist_control_mode;
  }

  public WristTask getWristTask() {
    return wrist_task;
  }

  public void drive_manually(double arm_speed, double wrist_speed) {
    // m_arm_state = ArmState.Moving;
    wrist_speed = MathUtil.applyDeadband(wrist_speed, .01);
    wrist_speed = MathUtil.clamp(wrist_speed, WRIST_REVERSE_SPEED, WRIST_FORWARD_SPEED);

    // System.out.println("Arm Speed: " + arm_speed);
    m_wrist_motor.set(ControlMode.PercentOutput, wrist_speed);
  }

  public void checkReverseLimits() {
    // falcon hard limit returns 1 if closed, 0 if open. Our limits are normally
    // open

    if (m_wrist_motor.isRevLimitSwitchClosed() == 1) {
      // System.out.println("Wrist Switch close");
      m_wrist_motor.setSelectedSensorPosition(0);
    }
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

  public void configure_wrist_motion_magic_test(double velocity, double time_to_velo, double kP, double kF, boolean isForward){
    // Dividing by zero is very bad, will crash most systems. 
   if( time_to_velo == 0 ){
     time_to_velo = 1;
   }
   double acceleration = velocity / time_to_velo;

   if (isForward ){
     m_wrist_motor.selectProfileSlot(WRIST_MM_FORWARD_SLOT, 0);
     m_wrist_motor.config_kP(WRIST_MM_FORWARD_SLOT, kP);
     m_wrist_motor.configMotionCruiseVelocity(velocity);
     m_wrist_motor.configMotionAcceleration(acceleration);
   } else {
     m_wrist_motor.selectProfileSlot(WRIST_MM_REVERSE_SLOT, 0);
     m_wrist_motor.config_kP(WRIST_MM_REVERSE_SLOT, kP);
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
  double curr_degrees = wristToDegrees(m_wrist_motor.getSelectedSensorPosition());
  return WRIST_FORWARD_COSINE_FF * (Math.cos(curr_degrees + WRIST_COSINE_STARTING_OFFSET));
}

public boolean is_wrist_mm_done(int target_ticks){
  double wrist_pos = m_wrist_motor.getSelectedSensorPosition();

  // m_wrist_mm_error.set(Math.abs(target_ticks - wrist_pos));
  return Math.abs((target_ticks - wrist_pos)) < WRIST_POSITION_TOLERANCE;
}

public boolean is_wrist_rev_limit_hit() {
  return m_wrist_motor.isRevLimitSwitchClosed() == 1;
}

public boolean is_wrist_fwd_limit_hit() {
  return m_wrist_motor.isFwdLimitSwitchClosed() == 1;
}

public void holdWristPosition(){
  double wrist_pos = m_wrist_motor.getSelectedSensorPosition();

  // Set the arm and wrist to their hold position slots as primary pids
  m_wrist_motor.selectProfileSlot(WRIST_HOLD_POSITION_SLOT, 0);

  // Set the motor to hold with PID
  m_wrist_motor.set(ControlMode.Position, wrist_pos, DemandType.ArbitraryFeedForward, getWristArbFF());
}

public void set_current_position_to_manual() {
  m_current_position = SuperStructurePosition.Manual;
}

public void configure_wrist_motion_magic(int target_ticks, boolean isForward){
  if( isForward ){
    m_wrist_motor.selectProfileSlot(WRIST_MM_FORWARD_SLOT, PID_PRIMARY);
    m_wrist_motor.configMotionCruiseVelocity(WRIST_MM_FORWARD_VELOCITY);
    m_wrist_motor.configMotionAcceleration(WRIST_MM_FORWARD_ACCELERATION);
  }
  else {
    m_wrist_motor.selectProfileSlot(WRIST_MM_REVERSE_SLOT, PID_PRIMARY);
    m_wrist_motor.configMotionCruiseVelocity(WRIST_MM_REVERSE_VELOCITY);
    m_wrist_motor.configMotionAcceleration(WRIST_MM_REVERSE_ACCELERATION); 
  }
}
public void checkArmSuperState() {
  if(m_wrist_motor.getSelectedSensorPosition() <= 500){
    m_current_position = SuperStructurePosition.Stowed;
  }
}

public boolean isBackwardMovement(SuperStructurePosition pos){
  double wristPos = m_wrist_motor.getSelectedSensorPosition();
  return  !(wristPos < pos.wrist_position);
}

public void set_state_to_inactive(){
  m_wrist_state = WristState.Inactive;
}

public CommandBase setWristEncoderToZero() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return runOnce(
      () -> {
        m_wrist_motor.setSelectedSensorPosition(0);
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

public CommandBase set_state(WristState state){
  return runOnce(
    () -> {
      m_wrist_state = state;
    });
}

}

