// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeEject;
import frc.robot.commands.IntakeGround;
import frc.robot.commands.IntakePulse;
import frc.robot.commands.IntakeStow;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.Shootback;
import frc.robot.commands.Shooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    protected SendableChooser<Command> autoChooser = new SendableChooser<>();
    // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shoot = new ShooterSubsystem();
  private final Shooter shooter = new Shooter();
  private final IntakeGround intakeground = new IntakeGround();
  private final IntakeStow intakestow = new IntakeStow();
 
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_controlController = new Joystick(OIConstants.kControlControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */ 

  private void configureAutoCommands(){
    NamedCommands.registerCommands(Map.of(
        "shootsequence", Commands.sequence(
            shooter.Shoot(m_shoot, m_intake)
        ),
        "intakeGround", Commands.sequence(
            intakeground.intakeground(m_intake)
        ),
        "intakeStow", Commands.sequence(
            intakestow.intakestow(m_intake)
        )
    ));
  }

  public RobotContainer() {
    //Must register commands used in PathPlanner autos
    NamedCommands.registerCommand("Shoot", new Shooter().Shoot(m_shoot, m_intake).withTimeout(1.5));
    NamedCommands.registerCommand("GroundIntake", new IntakeGround().intakeground(m_intake));
    NamedCommands.registerCommand("IntakeStow", new IntakeStow().intakestow(m_intake)); // We don't use the amp so deflector not needed
    
    //Creates sendable chooser for use with PathPlanner autos
    
    configureAutoCommands();
    // Configure the button binding
    configureButtonBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    configureAuto();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  private void configureAuto(){
    autoChooser.setDefaultOption("showauto", AutoBuilder.buildAuto("showauto"));
    autoChooser.addOption("redoutter", AutoBuilder.buildAuto("redoutter"));
    autoChooser.addOption("blueoutter", AutoBuilder.buildAuto("blueoutter"));
    autoChooser.addOption("blueinner", AutoBuilder.buildAuto("blueinner"));
    autoChooser.addOption("redinner", AutoBuilder.buildAuto("redinner"));
    
    
    SmartDashboard.putData("Auto Routine", autoChooser);
  }
  
  /*public Command BlueInner(){
    AutoBuilder.buildAuto("blueinner");
    return new PathPlannerAuto("blueinner");
  }*/
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, 1)
        .onTrue(new IntakeGround().intakeground(m_intake));

    new JoystickButton(m_driverController, 2)
        .onTrue(new IntakeCommand(m_intake, 0.23, 1));
    
    new JoystickButton(m_driverController, 4)
        .onTrue(new IntakeStow().intakestow(m_intake));

    new JoystickButton(m_driverController, 5)
        .whileTrue(new Shooter().Shoot(m_shoot, m_intake));
    
    new JoystickButton(m_controlController, 3)
        .whileTrue(new IntakePulse(m_intake));
    
    new JoystickButton(m_driverController, 3)
        .whileTrue(new ShootAmp().Amp(m_shoot, m_intake));
    
    new JoystickButton(m_controlController, 5)
        .whileTrue(new IntakeEject(m_intake));
    
    new JoystickButton(m_controlController, 7)
        .whileTrue(new Shootback(m_shoot));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(new Pose2d());

    return autoChooser.getSelected();
  }
  
}
