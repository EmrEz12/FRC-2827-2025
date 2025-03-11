// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ModifyingConstants;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intake;
  private int intakeTrack;
  private double pos;

  private double trackAngle;
  private double gearRatio;

  private double goal;
  private double rotationGoal;
  private double rotationError;

  /**
   * Creates a new SlideCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(IntakeSubsystem subsystem, double position, int track) {
    m_intake = subsystem;
    intakeTrack = track;
    pos = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(intakeTrack == 1) {
      trackAngle = ModifyingConstants.PIVOT_TRACK;
      gearRatio = ModifyingConstants.PIVOT_GEAR_RATIO;
    }
    

    goal = trackAngle * pos;
    rotationGoal = m_intake.AngleToRotations(goal, gearRatio);
  }
  
  /*else if (intakeTrack == 2) {
      trackAngle = ModifyingConstants.SLIDE_OUT_TRACK_LENGTH;
      gearRatio = ModifyingConstants.SLIDE_OUT_GEAR_RATIO;
    }*/
  
    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationError = (rotationGoal + m_intake.getMotorRotation(intakeTrack));
    m_intake.slideMove(m_intake.pidCalc(rotationError), intakeTrack);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.slideMove(0, 1);
    //m_intake.slideMove(0, 2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(rotationError) <= 0.2); // Might need adjusting
  }
}