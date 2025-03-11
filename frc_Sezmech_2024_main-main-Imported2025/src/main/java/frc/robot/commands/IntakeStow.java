// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStow {
  /** Static factory for an autonomous command. */
  public Command intakestow(IntakeSubsystem m_intake) {
    return Commands.sequence(
        new IntakeCommand(m_intake, 0.0, 1)
        );
  }
}