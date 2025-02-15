package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class ShuffleBoard {
    ShuffleboardTab m_tab;
    ShuffleboardLayout m_driveLay;
    public ShuffleBoard(DriveSubsystem driveSubsystem){
        //Competition
        m_tab = Shuffleboard.getTab("Competition");
        m_driveLay = m_tab.getLayout("Drive", BuiltInLayouts.kList);
        SmartDashboard.putData(driveSubsystem.getField());
        m_driveLay.add(driveSubsystem.getField())
            .withSize(28, 12)
            .withPosition(0, 0);
        m_driveLay.addDouble("Gyro Angle", ()->driveSubsystem.getGyro().getAngle())
            .withSize(4, 4)
            .withPosition(0, 12);
        m_driveLay.addDouble("OdometryX", ()->driveSubsystem.getPoseEstimator().getEstimatedPosition().getX())
            .withSize(4, 4)
            .withPosition(4, 12);
        m_driveLay.addDouble("OdometryY", ()->driveSubsystem.getPoseEstimator().getEstimatedPosition().getY())
            .withSize(4, 4)
            .withPosition(8, 12);
    }
}
