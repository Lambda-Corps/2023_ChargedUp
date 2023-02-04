// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UpdateSlewRateLimitersFromShuffleboard extends InstantCommand {
  DriveTrain m_drive_train;
  DoubleEntry m_forward_slew_rate_entry, m_turn_slew_rate_entry;

  public UpdateSlewRateLimitersFromShuffleboard(DriveTrain driveTrain) {
    m_drive_train = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive_train);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive_train.updateSlewRates();
  }
}
