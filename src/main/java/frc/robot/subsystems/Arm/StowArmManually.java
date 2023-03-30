// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist.Wrist;


public class StowArmManually extends CommandBase {
  private Arm m_arm;
  private Wrist m_wrist;
  private double armDriveSpeed;
  private boolean is_done;
  /** Creates a new GroundPickupArmDriveManually. */
  public StowArmManually(Arm arm, Wrist wrist) {
    m_arm = arm;
    m_wrist = wrist;
    armDriveSpeed = m_arm.ARM_REVERSE_SPEED;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    is_done = m_arm.getArmReverseLimit();
    if(m_arm.getArmReverseLimit() == true){
      m_arm.drive_manually(armDriveSpeed);
      m_wrist.drive_manually(0);
    }else if (m_arm.getArmReverseLimit() == false){
      m_arm.drive_manually(0);
      m_wrist.drive_manually(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( is_done == false){
      return true;
    }else {
      return false;
    }
    
  }
}
