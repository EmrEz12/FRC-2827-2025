// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeGround {
  /** Static factory for an autonomous command. */
  public Command intakeground(IntakeSubsystem m_intake) {
    return Commands.sequence(
        new IntakeCommand(m_intake, 0.55, 1),
        new IntakePulse(m_intake).until(m_intake.intakeHasNoteSupplier())
      );


  }
}