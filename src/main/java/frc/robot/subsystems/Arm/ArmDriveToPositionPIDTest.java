// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm.SuperStructurePosition;

public class ArmDriveToPositionPIDTest extends CommandBase {
  Arm m_arm;
  SuperStructurePosition m_position;
  int m_target_ticks;

  boolean m_done, m_direction_is_forward;
  int m_half_second_limit_hit;
  int m_count;

  NetworkTableEntry m_kPEntry, m_time_to_velo, m_target_velocity, m_target;
  /** Creates a new DriveToPosition. */
  public ArmDriveToPositionPIDTest(Arm arm, SuperStructurePosition position) {
    m_arm = arm;
    m_position = position;
    
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
    NetworkTable armTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Arm Test");
    m_kPEntry = armTab.getEntry("Arm kP");
    m_time_to_velo = armTab.getEntry("Time to Velo");
    m_target_velocity = armTab.getEntry("Target Velocity");
    m_target = armTab.getEntry("Target Ticks");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_done = false;
    m_half_second_limit_hit = 0;
    m_count = 0;
    // m_target_ticks = (int)(m_target.getDouble(SuperStructurePosition.Stowed.getArmPosition()));
    m_target_ticks = m_position.getArmPosition();
    m_arm.configure_arm_motion_magic_test(m_target_velocity.getDouble(0), m_time_to_velo.getDouble(1), m_kPEntry.getDouble(0));

    m_arm.move_arm_motion_magic(m_target_ticks);

    // m_direction_is_forward = (m_arm.getArmPosition()):

    m_direction_is_forward = (m_arm.getSuperStructureArmPosition() < m_position.getArmPosition());
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
      if( !m_direction_is_forward && m_arm.is_arm_rev_limit_hit()){
        m_arm.set_arm_encoder_to_zero();
      }

      m_arm.set_current_position(m_position);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_count > 5) || (m_half_second_limit_hit >= 25);
  }
}
