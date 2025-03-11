// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.ModifyingConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax motor_pivot;
  private final SparkMax motor_intake;
  

  private PIDController slidePID;
  private final double P = 0.9; //
  private final double I = 0.0; // Will need adjusting
  private final double D = 0.0; //

  private final DigitalInput m_IntakeLimitSwitch = new DigitalInput(Constants.ModifyingConstants.k_intakeLimitSwitchId);
  
  /** Creates a new SlideSubsystem. */
  public IntakeSubsystem() {
    motor_pivot = new SparkMax(IDConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    motor_intake = new SparkMax(IDConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  
    motor_intake.configure(Configs.IntakeSubsystem.intakeConfig , ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor_pivot.configure(Configs.IntakeSubsystem.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    slidePID = new PIDController(P, I, D);
  }

  public void pulse(){
    motor_intake.set(0.4);
  }

  public void eject(){
    motor_intake.set(-0.2);
  }

  public void Ampeject(){
    motor_intake.set(-0.3);
  }

  public void intakestop(){
   motor_intake.stopMotor();
  }

  public void slideMove(double slidePow, int slideTrack) {
    if(slideTrack == 1) {
      motor_pivot.set(slidePow * ModifyingConstants.PIVOT_COEFFICIENT);
    }
  }
  /*else if (slideTrack == 2) {
        motor_pivot.set(slidePow * ModifyingConstants.PIVOT_COEFFICIENT);
    }*/

  public double AngleToRotations(double angle, double gearRatio) {
    return (angle*gearRatio); // Calculated for a gear/wheel of radius 27.5 mm
  }

  public double getMotorRotation(int motorNum) {
    return motor_pivot.getEncoder().getPosition();
  }

  public double pidCalc(double err) {
    return slidePID.calculate(err);
  }

  public boolean getIntakeHasNote() {
     return !m_IntakeLimitSwitch.get();
    // NOTE: this is intentionally inverted, because the limit switch is normally
    // closed
  }
  public BooleanSupplier intakeHasNoteSupplier() {
    return this::getIntakeHasNote;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Power", motor_pivot.get());
    SmartDashboard.putBoolean("limit switch", !m_IntakeLimitSwitch.get());
    SmartDashboard.putNumber("Rotations", motor_intake.getEncoder().getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}