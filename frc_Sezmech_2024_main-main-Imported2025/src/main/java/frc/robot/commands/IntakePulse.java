package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

//import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class IntakePulse extends Command {
  
  private final IntakeSubsystem m_subsytem;
  //private final Intake m_intake;
  public IntakePulse(IntakeSubsystem subsystem){
    m_subsytem = subsystem;
    //m_intake = intake;
    
    
  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    m_subsytem.pulse();
  }

  @Override
  public void end(boolean interrupted) {
    m_subsytem.intakestop();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}