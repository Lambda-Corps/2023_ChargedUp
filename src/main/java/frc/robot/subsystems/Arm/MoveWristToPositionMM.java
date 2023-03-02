// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm.SuperStructurePosition;

public class MoveWristToPositionMM extends CommandBase {
  Arm m_arm;
  SuperStructurePosition m_position;
  int m_target_ticks;

  boolean m_done, m_direction_is_forward;
  int m_half_second_limit_hit;
  int m_count;

  /** Creates a new MoveWristToPositionMM. */
  public MoveWristToPositionMM(Arm arm, SuperStructurePosition position) {
    m_arm = arm;
    m_position = position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_done = false;
    m_half_second_limit_hit = 0;
    m_count = 0;
    // m_target_ticks = (int)(m_target.getDouble(SuperStructurePosition.Stowed.getArmPosition()));
    m_target_ticks = m_position.getArmPosition();
    m_direction_is_forward = m_arm.getSuperStructureWristPosition() < m_position.getWristPosition();
    m_arm.configure_wrist_motion_magic(m_target_ticks, m_direction_is_forward);

    m_arm.move_wrist_motion_magic(m_target_ticks, m_direction_is_forward);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_done = m_arm.is_arm_mm_done(m_target_ticks);
    if(m_done){
      m_count++;
    }
    else{
      m_count = 0;
    }
    
    if( m_direction_is_forward && m_arm.is_arm_fwd_limit_hit() ){
      m_half_second_limit_hit++;
    } else if( !m_direction_is_forward && m_arm.is_arm_rev_limit_hit() ){
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
      m_arm.holdPosition();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_count > 5) || (m_half_second_limit_hit >= 25);
  }
}
