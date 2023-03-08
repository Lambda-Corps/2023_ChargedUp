// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistanceInInchesTest extends CommandBase {
  DriveTrain m_dt;
  double m_target_ticks;
  boolean m_done;
  int m_count_done;
  NetworkTableEntry m_speed, m_targetticks, m_kPEntry, m_kdEntry, m_distanceEntry, m_setpoint_in_ticks_entry;

  /** Creates a new TurnToAngleWithGyro. */
  public DriveDistanceInInchesTest(DriveTrain dt) {
    m_dt = dt;
    addRequirements(m_dt);
    // Use addRequirements() here to declare subsystem dependencies.
    NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Test");
    m_kPEntry = driveTab.getEntry("MM kP");
    m_kdEntry = driveTab.getEntry("MM kD");
    m_distanceEntry = driveTab.getEntry("Target Distance");
    m_setpoint_in_ticks_entry = driveTab.getEntry("Target in Ticks");

    addRequirements(m_dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dt.reset_setpoints();
    m_dt.reset_encoders();
    m_done = false;
    m_count_done = 0;
    m_target_ticks = (int)(m_distanceEntry.getDouble(0)* DriveTrain.kEncoderTicksPerInch);

    m_setpoint_in_ticks_entry.setDouble(m_target_ticks);

    m_dt.configurePIDDrive( m_kPEntry.getDouble(0), m_kdEntry.getDouble(0), 0,m_target_ticks);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_dt.turn_target_degrees()) {
    //   m_count_done++;
    // } else {
    //   m_count_done = 0;
    // }
    m_done = m_dt.drive_straight_with_pid();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dt.teleop_drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_count_done > 5;
    return m_done;
  }
}
