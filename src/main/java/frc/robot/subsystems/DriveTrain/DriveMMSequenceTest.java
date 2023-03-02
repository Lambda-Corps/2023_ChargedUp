// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveMMSequenceTest extends SequentialCommandGroup {
  /** Creates a new DriveMMSequence. */
  public DriveMMSequenceTest(DriveTrain dt) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveMotionMagicTest(dt).raceWith(new WaitCommand(2)), dt.stopMotorsCommand(),
     // new WaitCommand(1),
      new TurnToAngleWithGyroTest(dt).raceWith(new WaitCommand(1.5)), dt.stopMotorsCommand());
  }
}
