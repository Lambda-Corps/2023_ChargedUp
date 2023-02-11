// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveMMInstant extends InstantCommand {
  DriveTrain m_dt;
  int m_target_in_ticks;
  boolean m_done;
  int m_count;
  
  NetworkTableEntry m_target_distance, m_time_to_velo, m_target_velocity, m_left_result, m_right_result, m_kp;
  public DriveMMInstant(DriveTrain dt) {
    m_dt = dt;
    
    NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Test");
    m_target_distance = driveTab.getEntry("Target Distance");
    m_time_to_velo = driveTab.getEntry("Time to Velo");
    m_target_velocity = driveTab.getEntry("Target Velocity");
    m_left_result = driveTab.getEntry("Left Encoder Result");
    m_right_result = driveTab.getEntry("Right Encoder Result");
    m_kp = driveTab.getEntry("MM kP");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dt);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_done = false;
    m_count = 0;
    m_target_in_ticks = (int)(m_target_distance.getDouble(0)* DriveTrain.kEncoderTicksPerInch);

    m_dt.configure_motion_magic( m_target_velocity.getDouble(0), m_time_to_velo.getDouble(0), m_kp.getDouble(0));
    m_dt.drive_motion_magic(m_target_in_ticks);
  }
}
