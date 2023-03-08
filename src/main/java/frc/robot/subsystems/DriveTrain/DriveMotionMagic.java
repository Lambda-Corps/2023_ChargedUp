// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveMotionMagic extends CommandBase {
  DriveTrain m_dt;
  double m_target_in_inches;
  int m_target_in_ticks;
  boolean m_done;
  int m_count;

  /** Creates a new DriveMotionMagic. */
  public DriveMotionMagic(DriveTrain dt, double distance_in_inches) {
    m_dt = dt;
    m_target_in_inches = distance_in_inches;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_target_in_ticks = (int)(m_target_in_inches * DriveTrain.kEncoderTicksPerInch);
    m_done = false;
    m_count = 0;

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
