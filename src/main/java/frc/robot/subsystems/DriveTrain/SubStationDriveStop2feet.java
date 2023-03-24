// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SubStationDriveStop2feet extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final CommandXboxController m_driver_controller;
  /** Creates a new SubStationDriveStop2feet. */
  private boolean is_done;
  private double drive_speed;
  public SubStationDriveStop2feet(DriveTrain dt, CommandXboxController xbox) {
    m_drivetrain = dt;
    m_driver_controller = xbox;

    drive_speed = 0.7; // 70% motor output
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double raw_turn_axis = -(m_driver_controller.getRawAxis(4));
    is_done = m_drivetrain.DriveUntil2FeetFromSubStation(drive_speed, raw_turn_axis);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.teleop_drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (is_done == true){
      return true;
    }
    else{
      return false;
    }
  }
}
