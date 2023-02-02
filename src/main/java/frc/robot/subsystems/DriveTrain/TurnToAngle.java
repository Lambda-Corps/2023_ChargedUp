// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToAngle extends CommandBase {
  /** Creates a new Turn90DegreesClockwiseWithGryo. */
  DriveTrain m_drive_train;
  AHRS m_gyro;
  Timer m_timer;
  DoubleEntry m_max_speed_entry, m_target_heading_entry, m_current_heading_entry;

  double m_initial_heading, m_current_heading;
  double command_timeout = 5;

  public TurnToAngle(DriveTrain driveTrain) {
    NetworkTableInstance nt_inst = NetworkTableInstance.getDefault();
		NetworkTable nt_table = nt_inst.getTable("Shuffleboard/Drive Test");
    m_max_speed_entry = new DoubleTopic(nt_table.getDoubleTopic("Max Speed")).getEntry(0, PubSubOption.keepDuplicates(true));

    m_drive_train = driveTrain;
    m_gyro = m_drive_train.getDriveTrainGyro();
    m_timer = new Timer();

    m_target_heading_entry = new DoubleTopic(nt_table.getDoubleTopic("Target Heading")).getEntry(0, PubSubOption.keepDuplicates(true));
    m_target_heading_entry = new DoubleTopic(nt_table.getDoubleTopic("Current Heading")).getEntry(0, PubSubOption.keepDuplicates(true));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_initial_heading = m_gyro.getAngle();

    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_current_heading = m_gyro.getAngle();
    // get the difference between the target and current heading. 
    double heading_dif = m_target_heading_entry.getAsDouble() - m_current_heading;
    double turn_value = m_max_speed_entry.getAsDouble() * (heading_dif);

    m_current_heading_entry.set(m_current_heading);
    m_drive_train.teleop_drive(0, turn_value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_current_heading == m_target_heading_entry.getAsDouble() || m_timer.hasElapsed(command_timeout);
  }
}
