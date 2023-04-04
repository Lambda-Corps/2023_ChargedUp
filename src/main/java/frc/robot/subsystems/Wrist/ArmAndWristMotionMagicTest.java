// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.MoveArmToPositionMM;
import frc.robot.subsystems.Arm.MoveWristToPositionMM;
import frc.robot.subsystems.Arm.Arm.ArmSuperStructurePosition;
import frc.robot.subsystems.Wrist.Wrist.WristSuperStructurePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmAndWristMotionMagicTest extends SequentialCommandGroup {
  Arm m_arm;
  Wrist m_wrist;
  /** Creates a new ReplaceMeSequentialCommandGroup. */
  public ArmAndWristMotionMagicTest(Arm arm, Wrist wrist, ArmSuperStructurePosition armposition, WristSuperStructurePosition wristposition) {
    m_arm = arm;
    m_wrist = wrist;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveWristToPositionMM(m_wrist, wristposition).raceWith(new WaitCommand(3).raceWith(new MoveArmToPositionMM(m_arm, armposition)).raceWith(new WaitCommand(3)))

    );
  }
}
