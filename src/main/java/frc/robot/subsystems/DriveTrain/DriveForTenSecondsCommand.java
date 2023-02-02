// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class DriveForTenSecondsCommand extends CommandBase {
  /** Creates a new DriveForTenSecondsCommand. */
  DriveTrain m_drive_train;
  Timer m_timer;
  DoubleEntry m_speed_entry;
  private double m_drive_speed;

  public DriveForTenSecondsCommand(DriveTrain driveTrain) {
    NetworkTableInstance nt_inst = NetworkTableInstance.getDefault();
		NetworkTable nt_table = nt_inst.getTable("Shuffleboard/Drive Test");
    m_speed_entry = new DoubleTopic(nt_table.getDoubleTopic("Max Speed")).getEntry(0, PubSubOption.keepDuplicates(true));

    m_timer = new Timer();

    m_drive_train = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_drive_speed = m_speed_entry.getAsDouble();

    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive_train.teleop_drive(m_drive_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive_train.teleop_drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(10);
  }
}
