package frc.robot.utils;

import java.util.EnumSet;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants.VirtualLimelightConstants;

/**
 * Allows simulating limelight in the simulator.
 * Requires hardcoding fiducials (april tags) to work
 */
public class VirtualLimelight {
    private final class Listener implements TableEventListener {
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.compareTo("camerapose_robotspace_set") == 0) {
                double[] camerapose_robotspace = getDoubleArrayEntry(m_table, "camerapose_robotspace_set").get();
                if (camerapose_robotspace.length == 6) {
                    m_camerapose_robotspace = camerapose_robotspace;
                }
            }

            else if (key.compareTo("imumode") == 0) {
                double imumode = getDoubleEntry(m_table, "imumode").get();
                if (imumode != -1) {
                    m_imumode = imumode;
                }
            }

            else if (key.compareTo("robot_orientation") == 0) {
                double[] robot_orientation = getDoubleArrayEntry(m_table, "robot_orientation").get();
                if (robot_orientation.length == 6) {
                    m_robot_orientation = robot_orientation;
                }
            }
        }
    }

    public final static class Fiducial {
        private final Pose2d m_pose;
        private double m_dx;
        private double m_dy;
        private double m_drot;
        private final double m_ambiguity;

        // all WPILib ecosystem position based, rotation in degrees
        public Fiducial(int id, double dx, double dy, double drot, double ambiguity) {
            m_dx = dx;
            m_dy = dy;
            m_drot = drot;
            m_ambiguity = ambiguity;

            final Pose3d pose3d = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
            m_pose = new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
        }

        /**
         * Shift deltas of the fiducials
         * @param dx
         * @param dy
         * @param drot
         */
        public void shift(double dx, double dy, double drot) {
            m_dx += dx;
            m_dy += dy;
            m_drot += drot;
        }

        public static double[] calculate(Fiducial[] fiducials) {
            double[] ret = new double[VirtualLimelightConstants.kbotpose_orb_wpiblue_header_size
                    + VirtualLimelightConstants.kValsPerFiducial * fiducials.length];

            /*
             * This method will create the botpose_orb packet. First is the
             * header with 11 doubles with the following structure:
             * 0: Robot position X
             * 1: Robot Position Y
             * 2: Reserved
             * 3: Reserved
             * 4: Reserved
             * 5: Robot rotation in degrees
             * 6: Latency
             * 7: Tag count
             * 8: Tag span
             * 9: Tag distance
             * 10:Tag Area (percentage of total area)
             * 
             * Only the robot position, rotation, latency, and tag count are
             * needed for pose estimation.
             * 
             * After the header, the rest of the space is for the fiducials.
             * Each fiducial should contain seven doubles, and the number of
             * fiducials should be equal to tc. Each fiducial should have the
             * following structure:
             * 0: Fiducial id
             * 1: Translation X (non converted)
             * 2: Translation Y (non converted)
             * 3: Rotation in degrees
             * 4: Distance to camera
             * 5: Distance to robot
             * 6: Ambiguity
             * 
             * Only the distance to camera and ambiguity are needed for pose
             * estimation.
             */

            // average pose
            double sx = 0;
            double sy = 0;
            double srot = 0;

            // populate packet
            int fiducialBase = VirtualLimelightConstants.kbotpose_orb_wpiblue_header_size;
            for (Fiducial f : fiducials) {
                sx += f.m_pose.getX() - f.m_dx;
                sy += f.m_pose.getY() - f.m_dy;
                srot += f.m_pose.getRotation().getDegrees() - f.m_drot;

                ret[fiducialBase + 4] = Math.sqrt(f.m_dx * f.m_dx + f.m_dy * f.m_dy);
                ret[fiducialBase + 6] = f.m_ambiguity;

                fiducialBase += VirtualLimelightConstants.kValsPerFiducial;
            }

            ret[0] = sx / fiducials.length;
            ret[1] = sy / fiducials.length;
            ret[5] = srot / fiducials.length;
            ret[6] = VirtualLimelightConstants.kLatency;
            ret[7] = fiducials.length;

            return ret;
        }
    }

    private final String m_table;

    @SuppressWarnings("unused")
    private double[] m_camerapose_robotspace = new double[6];
    @SuppressWarnings("unused")
    private double m_imumode = 0;
    @SuppressWarnings("unused")
    private double[] m_robot_orientation = new double[6];

    private final DoubleArrayPublisher m_camerapose_robotspace_pub;
    private final DoublePublisher m_imumode_pub;
    private final DoubleArrayPublisher m_robot_orientation_pub;
    private final DoubleArrayPublisher m_botpose_orb_wpiblue_pub;

    private final BooleanPublisher m_lock_pub;

    private Fiducial[] m_fiducials = new Fiducial[0];

    private final int m_listenerHandle;

    public VirtualLimelight(String name) {
        m_table = name;

        // check for real limelight. This works by seeing if NT has HW metric fps is -1
        // or nonexistant (virtual always sets this to -1)
        if (getDoubleArrayEntry("limelight", "hw").get(VirtualLimelightConstants.kHWMetrics)[0] != -1) {
            DriverStation.reportError("Running virtual limelight server alongside real hardware", false);
            throw new IllegalStateException("Cannot construct virtual limelight instance alongside real hardware");
        }

        // check for conflicting limelight names
        if (getBooleanEntry(m_table, "_lock").get()) {
            DriverStation.reportError("Running two virtual limelight servers with the same name", false);
            throw new IllegalStateException("Cannot construct duplicate virtual limelight instance");
        }

        // check if real
        if (Robot.isReal()) {
            // not an error because there is no physical limelight connected to NT. However,
            // this is probably still a bad idea
            DriverStation.reportWarning("Running virtual limelight on real robot", false);
        }

        m_camerapose_robotspace_pub = getDoubleArrayTopic(m_table, "m_camerapose_robotspace").publish();
        m_imumode_pub = getDoubleTopic(m_table, "m_imumode").publish();
        m_robot_orientation_pub = getDoubleArrayTopic(m_table, "m_robot_orientation").publish();
        m_botpose_orb_wpiblue_pub = getDoubleArrayTopic(m_table, "botpose_orb_wpiblue").publish();

        m_lock_pub = getBooleanTopic(m_table, "_lock").publish();
        m_lock_pub.set(true);

        m_listenerHandle = NetworkTableInstance.getDefault().getTable(m_table).addListener(EnumSet.of(Kind.kValueAll),
                new Listener());
    }

    public void setFiducials(Fiducial[] fiducials) {
        m_fiducials = fiducials;
    }

    public void update() {
        // write botpose_orb_wpiblue
        double[] botpose_orb_wpiblue = Fiducial.calculate(m_fiducials);

        // verify packet size
        if (botpose_orb_wpiblue.length != VirtualLimelightConstants.kbotpose_orb_wpiblue_header_size
                + VirtualLimelightConstants.kValsPerFiducial * botpose_orb_wpiblue[7]) {
            DriverStation.reportError("Bad botpose_orb_wpiblue data: wrong packet size", false);
            return;
        }

        m_botpose_orb_wpiblue_pub.set(botpose_orb_wpiblue, 0);

        NetworkTableInstance.getDefault().flush();
    }

    /**
     * Shift deltas of the robot (i.e. the negate of the fiducials)
     * @param dx
     * @param dy
     * @param drot
     */
    public void update(Pose2d botpose) {
        //TODO
        update();
    }

    public void close() {
        NetworkTableInstance.getDefault().removeListener(m_listenerHandle);

        m_lock_pub.set(false);

        m_camerapose_robotspace_pub.close();
        m_imumode_pub.close();
        m_robot_orientation_pub.close();
        m_botpose_orb_wpiblue_pub.close();
    }

    private static BooleanEntry getBooleanEntry(String table, String entry) {
        return getBooleanTopic(table, entry).getEntry(false);
    }

    private static DoubleEntry getDoubleEntry(String table, String entry) {
        return getDoubleTopic(table, entry).getEntry(-1);
    }

    private static DoubleArrayEntry getDoubleArrayEntry(String table, String entry) {
        return getDoubleArrayTopic(table, entry).getEntry(new double[0]);
    }

    private static BooleanTopic getBooleanTopic(String table, String entry) {
        return NetworkTableInstance.getDefault().getTable(table).getBooleanTopic(entry);
    }

    private static DoubleTopic getDoubleTopic(String table, String entry) {
        return NetworkTableInstance.getDefault().getTable(table).getDoubleTopic(entry);
    }

    private static DoubleArrayTopic getDoubleArrayTopic(String table, String entry) {
        return NetworkTableInstance.getDefault().getTable(table).getDoubleArrayTopic(entry);
    }
}