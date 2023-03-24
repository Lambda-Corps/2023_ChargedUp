// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLEDs extends InstantCommand {
  LED m_led;
  int m_strip;
  int m_color;
  public SetLEDs(LED led , int Strip, int color) {
    m_led = led;
    m_strip = Strip;
    m_color = color;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.setLED(m_strip, m_color);
  }
}
