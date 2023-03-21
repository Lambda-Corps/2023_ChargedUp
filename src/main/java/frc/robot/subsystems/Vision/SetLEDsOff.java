// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLEDsOff extends InstantCommand {
  Vision m_vision;
  public SetLEDsOff(Vision vision) {
    m_vision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.setLimelightLEDmode(false);
  }
}
