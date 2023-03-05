// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveMotionMagicTest extends CommandBase {
  DriveTrain m_dt;
  int m_target_in_ticks;
  boolean m_done;
  int m_count;
  NetworkTableEntry m_target_distance, m_time_to_velo, m_target_velocity, m_left_result, m_right_result, m_kp, m_setpoint_in_ticks;

  /** Creates a new DriveMotionMagic. */
  public DriveMotionMagicTest( DriveTrain dt) {
    m_dt = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dt);

    NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Test");
    m_target_distance = driveTab.getEntry("Target Distance");
    m_time_to_velo = driveTab.getEntry("Time to Velo");
    m_target_velocity = driveTab.getEntry("Target Velocity");
    m_left_result = driveTab.getEntry("Left Encoder Result");
    m_right_result = driveTab.getEntry("Right Encoder Result");
    m_kp = driveTab.getEntry("MM kP");
    m_setpoint_in_ticks = driveTab.getEntry("Target in Ticks");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_dt.reset_setpoint();
    // m_dt.reset_encoders();
    m_done = false;
    m_count = 0;
    m_target_in_ticks = (int)(m_target_distance.getDouble(0)* DriveTrain.kEncoderTicksPerInch);

    m_setpoint_in_ticks.setDouble(m_target_in_ticks);

    m_dt.configure_motion_magic_test( m_target_velocity.getDouble(0), m_time_to_velo.getDouble(0), m_kp.getDouble(0));
    m_dt.configure_motion_magic(m_target_in_ticks);
    m_dt.drive_motion_magic();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_done = m_dt.is_drive_mm_done();
    if(m_done){
      m_count++;
    }
    else{
      m_count = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dt.teleop_drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_count > 5;
  }
}
