// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveTrain.DefaultDriveTrainCommand;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

  private SendableChooser<Command> m_auto_chooser;
  private double m_left_speed, m_right_speed;
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_auto_chooser = new SendableChooser<Command>();
    // Set subsystem default commands
    m_drivetrain.setDefaultCommand(new DefaultDriveTrainCommand(m_drivetrain, m_driver_controller));
    
    // Build up the driver's heads up display
    buildShuffleBoard();

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
  private void buildShuffleBoard() {
    buildDriverTab();
    buildDriverTestTab();
  }
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_auto_chooser.getSelected();
  }
  
  private void buildDriverTab() {
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

    driveTab.add("Gyro", m_drivetrain.m_gyro).withSize(2, 2).withPosition(2, 0);

    //left and right outputs of joysticks
    driveTab.add("Left Output", 0).withSize(1, 1).withPosition(2, 2).withWidget(BuiltInWidgets.kDial)
                                  .withProperties(Map.of("Min", -1, "Max", 1));
    driveTab.add("Right Output", 0).withSize(1, 1).withPosition(3, 2).withWidget(BuiltInWidgets.kDial)
                                  .withProperties(Map.of("Min", -1, "Max", 1));
    // Establishes a Autonomous Descion tab use .addOption to make Autonomous options               
    driveTab.add("Autonomous Chooser", m_auto_chooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 2).withSize(2, 1);
    //Autonomous Options
    m_auto_chooser.setDefaultOption("Default Autonomous" , new PrintCommand("You have run the Autonomous Command!"));
    // Left and Right motor speeds for testing and correction
    driveTab.add("Right motor speed", m_right_speed).withPosition(4,0).withSize(2,2).withWidget(BuiltInWidgets.kGraph);
    driveTab.add("Left motor speed", m_left_speed).withPosition(6,0).withSize(2,2).withWidget(BuiltInWidgets.kGraph);
  }

  private void buildDriverTestTab() {
    ShuffleboardTab driveMMTab = Shuffleboard.getTab("Drive Testing");

    // Result Values on row 2
    driveMMTab.add("Tgt. Ticks", 0)                                          .withPosition(0, 1);
    driveMMTab.addNumber("Left Encoder", m_drivetrain::getLeftEncoderValue)  .withPosition(1, 1);
    driveMMTab.addNumber("Right Encoder", m_drivetrain::getRightEncoderValue).withPosition(2, 1);
    driveMMTab.addNumber("Gyro Read", m_drivetrain::getRawAngle)             .withPosition(3, 1);
    driveMMTab.add("Run Time", 0)                                            .withPosition(4, 1);
  }
  
}