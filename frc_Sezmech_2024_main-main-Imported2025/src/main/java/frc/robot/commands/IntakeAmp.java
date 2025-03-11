package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

//import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class IntakeAmp extends Command {
  
  private final ShooterSubsystem m_subsytem;
  //private final Intake m_intake;
  public IntakeAmp(ShooterSubsystem subsystem){
    m_subsytem = subsystem;
    //m_intake = intake;
    
    
  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    m_subsytem.run(0.18);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsytem.motorStop();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}