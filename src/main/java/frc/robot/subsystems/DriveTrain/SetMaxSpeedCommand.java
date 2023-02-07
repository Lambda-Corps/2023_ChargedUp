// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetMaxSpeedCommand extends CommandBase {
  private DriveTrain m_DT;
  /** Creates a new SetMaxSpeedCommand. */
  public SetMaxSpeedCommand(DriveTrain dTrain) {
    m_DT = dTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DT.reset_max_speed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
