// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DriveTrain.DefaultDriveTrainCommand;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveWithMotionMagic;
import frc.robot.subsystems.DriveTrain.SetMaxSpeedCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Human interface
  private final XboxController m_driver_controller = new XboxController(0);

  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_drivetrain = new DriveTrain();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final int m_target;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    buildDriveTestTab();
    // Set subsystem default commands
    m_drivetrain.setDefaultCommand(new DefaultDriveTrainCommand(m_drivetrain, m_driver_controller));
    m_target = 81920;
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("Auto Needs to be Fixed");
  }

  private void buildDriveTestTab() {
    ShuffleboardTab driveTestTab = Shuffleboard.getTab("Drive Test");
    driveTestTab.add("Drive_MM", m_drivetrain.driveMotionMagic(m_target)).withPosition(4, 0);
    driveTestTab.add("Target Motion Magic", m_target);
    driveTestTab.add("Motion Magic Drive", new DriveWithMotionMagic(m_drivetrain, m_target));

    driveTestTab.add("Drivetrain", m_drivetrain).withPosition(5, 2);
    // driveTestTab.add("Turn_MM", m_drivetrain.TurnToAngleMM(m_target)).withPosition();

    driveTestTab.add("Right Encoder", 0).withPosition(2, 0).withSize(1, 1);
    driveTestTab.add("Left Encoder", 0).withPosition(3, 0).withSize(1, 1);
    driveTestTab.add("Right Speed", 0).withPosition(2, 1).withSize(1,1);
    driveTestTab.add("Left Speed", 0).withPosition(3, 1).withSize(1,1);
    
    // Set the max speed variables
    driveTestTab.add("Max Speed", 0).withPosition(0, 2).withSize(1,1);
    //driveTestTab.add("Reset Max Speed", m_drivetrain.setMaxValue()).withPosition(0, 2).withSize(1, 1);
    // Set the max speed variables
    driveTestTab.addDouble("Current Speed", m_drivetrain::get_max_speed).withPosition(0, 3).withSize(1,1);
    driveTestTab.add("Set_Max Speed", new SetMaxSpeedCommand(m_drivetrain)).withPosition(1, 3).withSize(2, 1);

    //driveTestTab.add("Robot Heading", 0).withPosition(0, 0).withSize(2, 2).withWidget(BuiltInWidgets.kGyro);
    
    driveTestTab.add("kP", 0).withPosition(4, 0).withSize(1, 1);
    driveTestTab.add("kD", 0).withPosition(5, 0).withSize(1, 1);
    driveTestTab.add("kI", 0).withPosition(6, 0).withSize(1, 1);
    driveTestTab.add("kF", 0).withPosition(7, 0).withSize(1, 1);
    driveTestTab.add("Tgt. Ticks", 0 ).withPosition( 1, 4);
    driveTestTab.add("Tgt. Inches", 0 ).withPosition( 1, 5);
    driveTestTab.add("Run Time", 0 ).withPosition( 1, 6);
    driveTestTab.add("Finish Iterrations", 0 ).withPosition( 1, 7);
  }
}
