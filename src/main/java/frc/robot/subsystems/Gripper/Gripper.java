// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Gripper extends SubsystemBase {
  TalonSRX m_leftside, m_rightside;
  DoubleSolenoid m_gripper;

  DoubleSolenoid.Value m_requested_position, m_current_position;
  
  final DoubleSolenoid.Value GRIPPER_EXPAND = DoubleSolenoid.Value.kReverse;
  final DoubleSolenoid.Value GRIPPER_CONTRACT = DoubleSolenoid.Value.kForward;

  final double INTAKE_CONE_SPEED = .5;
  final double INTAKE_CUBE_SPEED = .5;
  final double INTAKE_HOLD_SPEED = .2;
  final double INTAKE_SCORE_SPEED = -.5;

  /** Creates a new Gripper. */
  public Gripper() {
    m_leftside = new TalonSRX(GRIPPER_LEFT_MOTOR);
    m_rightside = new TalonSRX(GRIPPER_RIGHT_MOTOR);

    m_leftside.configFactoryDefault();
    m_rightside.configFactoryDefault();

    // Left will be the leader, right will be inverted so that in each case forward ejects objects and 
    // reverse brings it in
    // m_rightside.setInverted(true);
    // m_rightside.follow(m_leftside);
    m_leftside.setInverted(true);

    m_gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GRIPPER_SOLENOID_CHANNEL_A,
    GRIPPER_SOLENOID_CHANNEL_B);
    // Set the gripper to contracted for our preload
    m_requested_position = GRIPPER_CONTRACT;
    m_current_position = m_requested_position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // If the state of the gripper is requested to be different, then change it
    if( m_requested_position != m_current_position ){
      m_gripper.set(m_requested_position);
      m_current_position = m_requested_position;
    }
  }

  private void runMotors(double speed){
    m_leftside.set(ControlMode.PercentOutput, speed);
    m_rightside.set(ControlMode.PercentOutput, speed);
  }

  private void request_gripper_position(DoubleSolenoid.Value pos){
    m_requested_position = pos;
  }

  public void close_gripper(){
    request_gripper_position(GRIPPER_CONTRACT);
  }

  public void open_gripper() {
    request_gripper_position(GRIPPER_EXPAND);
  }

  private void hold_game_piece() {
    m_leftside.set(ControlMode.PercentOutput, .3);
    m_rightside.set(ControlMode.PercentOutput, INTAKE_HOLD_SPEED);
  }

  public void eject_piece(){
    runMotors(INTAKE_SCORE_SPEED);
  }

  public void intake_piece(){
    runMotors(INTAKE_CONE_SPEED);
  }

  public void stop_motors(){
    runMotors(0);
  }

  public void full_speed_eject(){
    runMotors(-1);
  }
  /////////////////////////////////////// Inline Commands Go below here ////////////////////
  public CommandBase expandGripperCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_gripper.set(GRIPPER_EXPAND);
        });
  }
  public CommandBase JustexpandGripper() {
    // Expand Gripper wthout running motors
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_gripper.set(GRIPPER_EXPAND);
          runMotors(0);
        });
  }

  public CommandBase contractGripperCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_gripper.set(GRIPPER_CONTRACT);
        });
  }

  public CommandBase intakeGamePieceCommand() {
    return runOnce(
      () -> { 
        runMotors(-INTAKE_CONE_SPEED);
      }
    );
  }
  public CommandBase holdGamePieceCommand() {
    return runOnce( 
      ()-> {
        hold_game_piece();
      }
    );
  }
  public CommandBase scoreGamePieceCommand() {
    return runOnce(
      () -> { 
        runMotors(INTAKE_SCORE_SPEED);
      }
    );
  }
  public CommandBase stopIntakeMotorsCommand() {
    return runOnce(
      () -> { 
        runMotors(0);
      }
    );
  }

}
