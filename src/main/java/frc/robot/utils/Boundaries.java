// create speed limitations for the robot.
// create rectangle boundary from constants that the robot must stay within.
package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.auton.RedRight;
import frc.robot.subsystems.DriveSubsystem;

import java.awt.Point;

/** Add your docs here. */
public class Boundaries {
    private final double m_leftX;
    private final double m_rightX;
    private final double m_topY;
    private final double m_bottomY;
    private final DriveSubsystem m_driveSubsystem;
    private Pose2d currentPose;

    public Boundaries(DriveSubsystem driveSubsystem, double leftX, double rightX, double topY, double bottomY) {
        m_driveSubsystem = driveSubsystem;
        m_leftX = leftX;
        m_rightX = rightX;
        m_bottomY = bottomY;
        m_topY = topY;
    }

    public void fastPeriodic() {
        currentPose = m_driveSubsystem.getPose();
        MathUtil.clamp(currentPose.getX(), m_leftX, m_rightX);        
    }

    } 