/*
 * Zaki 2/12/24 your code is not working better look at it
 */

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    public static CANSparkMax Arm;
    public static RelativeEncoder Encoder;

    public ArmSubsystem() {
        Arm = new CANSparkMax(Constants.DriveConstants.armSpark, MotorType.kBrushless);
        Encoder = Arm.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public void setArm(double v) {
        Arm.setVoltage(v);
    }
}
