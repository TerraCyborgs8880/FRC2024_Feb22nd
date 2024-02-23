package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;

    private Timer timer = new Timer();
    private double setTime = 0.0;
    private boolean reachedDistance = false;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");

        timer.start();

    }



    public double estimateDistance(double limelightMountAngleDegrees, double limelightLensHeightInches, double goalHeightInches) {
        NetworkTable _table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry _ty = _table.getEntry("ty");
        double targetOffsetAngle_Vertical = _ty.getDouble(0.0);
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    
        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }
}
