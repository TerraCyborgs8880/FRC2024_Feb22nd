/*
 * Anzar 2/16/24 stinky stinky code, just like all java code
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeOutakeSubsystem extends SubsystemBase {
    public static CANSparkMax FrontIntakeMotor;
    public static CANSparkMax BackIntakeMotor;
    public static CANSparkMax WristMotor;

    Timer timer = new Timer();
    public static boolean toggleIntake = false;
    
    public IntakeOutakeSubsystem() {
        BackIntakeMotor = new CANSparkMax(Constants.DriveConstants.intakeSpark, MotorType.kBrushless);
        FrontIntakeMotor = new CANSparkMax(Constants.DriveConstants.outakeSpark, MotorType.kBrushless);
        WristMotor = new CANSparkMax(Constants.DriveConstants.wristSpark, MotorType.kBrushless);
        timer.start();
    }

    
    public void periodic() {
        if(toggleIntake) {
            setFrontIntake( Constants.DriveConstants.intakeOutakeVolts/10);//constant is tested
            setBackIntake(Constants.DriveConstants.intakeOutakeVolts/3);
        }

        else { 
            setFrontIntake(0);
            setBackIntake(0);
        }
        
       
        
        // if (timer.get() < 3.0) {
        //     setFrontIntake(Constants.DriveConstants.intakeOutakeVolts);
        //     if (timer.get() < 1.0) {
        //         setBackIntake(Constants.DriveConstants.intakeOutakeVolts/3.0);
        //     }
        // }
        // else {
        //     setFrontIntake(0); setBackIntake(0);
        //     timer.stop(); timer.reset();
        // }
    }

    public void setFrontIntake(double v) {
        FrontIntakeMotor.setVoltage(v);
    }

    public void setBackIntake(double v) {
        BackIntakeMotor.setVoltage(v);
    }

    public void setWrist(double v) {
        WristMotor.setVoltage(v);
    }

    public void intakeOn(double v) {
        setFrontIntake(v);
        setBackIntake(v);
    }
}
