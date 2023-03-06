// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceBangBangTestCommand extends CommandBase {
  /** Creates a new BalanceBangBangCommand. */
   NetworkTableEntry m_fwd_speed_entry, m_back_speed_entry, m_setpoint_entry, m_fwd_out, m_rev_out, m_pitch_entry;
   double m_fwd_speed, m_reverse_speed, m_setpoint, m_output, m_pitch;
   boolean m_done;
   int m_count;

   DriveTrain m_dt;
  public BalanceBangBangTestCommand(DriveTrain dt) {
    m_dt = dt;

    NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Test");
    m_fwd_speed_entry = driveTab.getEntry("Bang Bang Forward Speed");
    m_back_speed_entry = driveTab.getEntry("Bang Bang Reverse Speed");
    m_setpoint_entry = driveTab.getEntry("Bang Bang Target Pitch");
    m_fwd_out = driveTab.getEntry("Fwd Out");
    m_rev_out = driveTab.getEntry("Rev Out");
    m_pitch_entry = driveTab.getEntry("Bang Bang Test Pitch Value");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_fwd_speed = m_fwd_speed_entry.getDouble(0);
    m_reverse_speed = m_back_speed_entry.getDouble(0);
    m_setpoint = m_setpoint_entry.getDouble(0);

    m_dt.configure_forward_bangbang_controller(m_setpoint);
    m_dt.configure_reverse_bangbang_controller(-m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pitch = m_pitch_entry.getDouble(0);

    // double out = m_dt.drive_bang_bang(m_setpoint);
    m_rev_out.setDouble(m_dt.get_rev_bang_bang(m_pitch));

    // double m_output = (m_fwd_speed * m_dt.test_fwd_bang_bang(m_pitch) + (m_reverse_speed * m_dt.test_rev_bang_bang(m_pitch)));

    double out = m_dt.drive_bang_bang(m_fwd_speed, m_reverse_speed);
    m_fwd_out.setDouble(out);
    if( out == 0){
      m_count++;
    } else {
      m_count = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_count >= 20;
  }
}
