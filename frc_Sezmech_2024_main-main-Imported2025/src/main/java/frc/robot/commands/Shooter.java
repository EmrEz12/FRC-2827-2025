// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shooter {
  /** Static factory for an autonomous command. */
  public Command Shoot(ShooterSubsystem m_shoot, IntakeSubsystem m_intake) {
    return Commands.sequence(
      new ShootCommand(m_shoot).alongWith(new WaitCommand(0.5).andThen(new IntakeEject(m_intake)))        
      
    );
  }

}