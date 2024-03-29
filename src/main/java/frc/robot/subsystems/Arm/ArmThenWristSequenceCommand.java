// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm.ArmState;
import frc.robot.subsystems.Arm.Arm.ArmSuperStructurePosition;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.Wrist.WristState;
import frc.robot.subsystems.Wrist.Wrist.WristSuperStructurePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmThenWristSequenceCommand extends SequentialCommandGroup {
  /** Creates a new ArmThenWristSequenceCommand. */
  public ArmThenWristSequenceCommand(Arm arm, Wrist wrist, ArmSuperStructurePosition armposition, WristSuperStructurePosition wristposition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      arm.set_state(ArmState.Moving),
      wrist.set_state(WristState.Moving),
      new MoveArmToPositionMM(arm, armposition).withTimeout(3),
      new MoveWristToPositionMM( wrist, wristposition).withTimeout(3),
      arm.set_state(ArmState.Holding),
      wrist.set_state(WristState.Holding)
    );
  }
}
