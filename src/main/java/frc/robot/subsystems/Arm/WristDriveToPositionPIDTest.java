// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm.ArmState;
import frc.robot.subsystems.Arm.Arm.SuperStructurePosition;
import frc.robot.subsystems.Wrist.Wrist;

public class WristDriveToPositionPIDTest extends CommandBase {
  Arm m_arm;
  Wrist m_wrist;
  SuperStructurePosition m_position;
  int m_target_ticks, m_half_second_limit_hit;

  boolean m_done, m_is_forward_movement;
  int m_count;

  NetworkTableEntry m_kPEntry, m_kFEntry, m_time_to_velo, m_target_velocity, m_target;
  /** Creates a new DriveToPosition. */
  public WristDriveToPositionPIDTest(Arm arm, Wrist wrist, SuperStructurePosition position) {
    m_arm = arm;
    m_wrist = wrist;
    m_position = position;
    
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
    NetworkTable armTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Arm Test");
    m_kPEntry = armTab.getEntry("Wrist kP");
    m_kFEntry = armTab.getEntry("Wrist kF");
    m_target_velocity = armTab.getEntry("Wrist Velo");
    m_target = armTab.getEntry("Target Ticks");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_done = false;
    m_count = 0;
    m_half_second_limit_hit = 0;
    m_is_forward_movement = m_wrist.getSuperStructureWristPosition() < m_position.getWristPosition();
    // m_target_ticks = (int)(m_target.getDouble(SuperStructurePosition.Stowed.getArmPosition()));
    m_target_ticks = m_position.getWristPosition();
    m_wrist.configure_wrist_motion_magic_test(m_target_velocity.getDouble(10000), 
                                            1, 
                                            m_kPEntry.getDouble(1.6), 
                                            m_kFEntry.getDouble(0.17),
                                            m_is_forward_movement);

    m_wrist.move_wrist_motion_magic(m_target_ticks, m_is_forward_movement);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_done = m_wrist.is_wrist_mm_done(m_target_ticks);
    if(m_done){
      m_count++;
    }
    else{
      m_count = 0;
    }

    if( m_is_forward_movement && m_wrist.is_wrist_fwd_limit_hit() ){
      m_half_second_limit_hit++;
    } else if( !m_is_forward_movement && m_wrist.is_wrist_rev_limit_hit() ){
      m_half_second_limit_hit++;
    }
    else{
      m_half_second_limit_hit = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if( !interrupted ){
      m_arm.set_current_position(m_position);
      m_arm.holdArmPosition();
      m_wrist.holdWristPosition();
      m_arm.set_state(ArmState.Holding);
    }
    else {
      m_arm.set_state_to_inactive();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_count > 5) || (m_half_second_limit_hit >= 25);
  }
}
