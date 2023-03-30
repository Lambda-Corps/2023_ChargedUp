// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm.ArmState;
import frc.robot.subsystems.Arm.Arm.ArmSuperStructurePosition;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.Wrist.WristState;
import frc.robot.subsystems.Wrist.Wrist.WristSuperStructurePosition;

public class ArmDriveToPositionPIDTest extends CommandBase {
  Arm m_arm;
  Wrist m_wrist;
  ArmSuperStructurePosition m_arm_position;
  WristSuperStructurePosition m_wrist_position;
  int m_arm_target_ticks;
  int m_wrist_target_ticks;

  boolean m_done, m_arm_direction_is_forward, m_wrist_direction_is_forward;
  int m_half_second_limit_hit;
  int m_count;

  NetworkTableEntry m_kPEntry, m_kFEntry, m_velo, m_target;
  /** Creates a new DriveToPosition. */
  public ArmDriveToPositionPIDTest(Arm arm, Wrist wrist, ArmSuperStructurePosition armposition, WristSuperStructurePosition wristposition) {
    m_arm = arm;
    m_wrist = wrist;
    m_arm_position = armposition;
    m_wrist_position = wristposition;
    
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
    NetworkTable armTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Arm Test");
    m_kPEntry = armTab.getEntry("Arm kP");
    m_kFEntry = armTab.getEntry("Arm kF");

    m_velo = armTab.getEntry("Arm Velo");
    m_target = armTab.getEntry("Target Ticks");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.set_state(ArmState.Moving);
    m_done = false;
    m_half_second_limit_hit = 0;
    m_count = 0;
    // m_target_ticks = (int)(m_target.getDouble(SuperStructurePosition.Stowed.getArmPosition()));
    m_arm_target_ticks = m_arm_position.getArmPosition();
    m_wrist_target_ticks = m_wrist_position.getWristPosition();
    m_arm.configure_arm_motion_magic_test(m_velo.getDouble(1750), 
                                          1,
                                          m_kPEntry.getDouble(2.1), 
                                          m_kFEntry.getDouble(0.67));

    m_arm.move_arm_motion_magic(m_arm_target_ticks);

    // m_direction_is_forward = (m_arm.getArmPosition()):

    m_arm_direction_is_forward = (m_arm.getSuperStructureArmPosition() < m_arm_position.getArmPosition());
    m_wrist_direction_is_forward = (m_wrist.getSuperStructureWristPosition() < m_wrist_position.getWristPosition());
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_done = (m_arm.is_arm_mm_done(m_arm_target_ticks) & m_wrist.is_wrist_mm_done(m_wrist_target_ticks));
    if(m_done){
      m_count++;
    }
    else{
      m_count = 0;
    }
    
    if( m_arm_direction_is_forward && m_arm.is_arm_fwd_limit_hit() ){
      m_half_second_limit_hit++;
    } else if( !m_arm_direction_is_forward && m_arm.is_arm_rev_limit_hit() ){
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
      if( !m_arm_direction_is_forward && m_arm.is_arm_rev_limit_hit()){
        m_arm.set_arm_encoder_to_zero();
      }
      m_arm.set_current_position(m_arm_position);
      m_wrist.set_current_position(m_wrist_position);
      m_arm.set_state(ArmState.Holding);
      m_wrist.set_state(WristState.Holding);
      m_arm.holdArmPosition();
      m_wrist.holdWristPosition();
    }else {
      m_arm.set_state(ArmState.Inactive);
      m_arm.drive_manually(0);
      m_wrist.drive_manually(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_count > 5) || (m_half_second_limit_hit >= 25);
  }
}
