// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SubstationPickupDistanceRangefinder extends CommandBase {
  private final DriveTrain m_drivetrain;
 
  private double drive_speed;
  private boolean is_done;

  public SubstationPickupDistanceRangefinder(DriveTrain dt) {
    m_drivetrain = dt;

    drive_speed = 0.25; // 20% motor output
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    is_done = m_drivetrain.DriveUntilSubstationPickup(drive_speed, 0);
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
