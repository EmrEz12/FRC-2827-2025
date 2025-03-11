package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;



public class ShooterSubsystem {
    private static SparkMax shoot_motor_2, shoot_motor_1;
    //private static WPI_VictorSPX shoot_motor_2, shoot_motor_1;

    public ShooterSubsystem(){
    shoot_motor_1 = new SparkMax(11, MotorType.kBrushless);
    shoot_motor_2 = new SparkMax(12, MotorType.kBrushless);
    
    //shoot_motor_1 = new WPI_VictorSPX(0);
    //shoot_motor_2 = new WPI_VictorSPX(1);
    }

    public void run(double speed){
        shoot_motor_1.set(-speed);
        shoot_motor_2.set(speed);
    }

    public void motorStop(){
        shoot_motor_1.stopMotor();
        shoot_motor_2.stopMotor();
    }
    
}
