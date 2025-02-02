package frc.robot.utils;

import java.util.EnumSet;

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

    private class Listener implements TableEventListener {
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

    public void update() {
        // write botpose_orb_wpiblue
        final double tx = 0; // translation x
        final double ty = 0; // translation y
        final double rd = 0; // rotation in degrees
        final double lt = 0.1; // latency
        final double tc = 0; // tag count
        final double ts = 0; // tag span
        final double td = 0; // tag distance
        final double ta = 0; // tag area (percentage of total area)

        double[] botpose_orb_wpiblue = new double[] {
                tx,
                ty,
                0,
                0,
                0,
                rd,
                lt,
                tc,
                ts,
                td,
                ta,
                /*
                 * After the header, the rest of the space is for the fiducials.
                 * Each fiducial should contain seven doubles, and the number of
                 * fiducials should be equal to tc. Each fiducial should have the
                 * following structure:
                 * 0: Fiducial id
                 * 1: Translation X (non converted)
                 * 2: Translation Y (non converted)
                 * 3: Rotation in Degrees
                 * 4: Distance to Camera
                 * 5: Distance to Robot
                 * 6: Ambiguity
                 */
        };

        if (botpose_orb_wpiblue.length != VirtualLimelightConstants.kbotpose_orb_wpiblue_header_size
                + VirtualLimelightConstants.kValsPerFiducial * tc) {
            DriverStation.reportError("Bad botpose_orb_wpiblue data: wrong packet size", false);
            return;
        }

        m_botpose_orb_wpiblue_pub.set(botpose_orb_wpiblue, 0);

        NetworkTableInstance.getDefault().flush();
    }

    public void close() {
        NetworkTableInstance.getDefault().removeListener(m_listenerHandle);

        m_lock_pub.set(false);

        m_camerapose_robotspace_pub.close();
        m_imumode_pub.close();
        m_robot_orientation_pub.close();
        m_botpose_orb_wpiblue_pub.close();
    }

    private BooleanEntry getBooleanEntry(String table, String entry) {
        return getBooleanTopic(table, entry).getEntry(false);
    }

    private DoubleEntry getDoubleEntry(String table, String entry) {
        return getDoubleTopic(table, entry).getEntry(-1);
    }

    private DoubleArrayEntry getDoubleArrayEntry(String table, String entry) {
        return getDoubleArrayTopic(table, entry).getEntry(new double[0]);
    }

    private BooleanTopic getBooleanTopic(String table, String entry) {
        return NetworkTableInstance.getDefault().getTable(table).getBooleanTopic(entry);
    }

    private DoubleTopic getDoubleTopic(String table, String entry) {
        return NetworkTableInstance.getDefault().getTable(table).getDoubleTopic(entry);
    }

    private DoubleArrayTopic getDoubleArrayTopic(String table, String entry) {
        return NetworkTableInstance.getDefault().getTable(table).getDoubleArrayTopic(entry);
    }
}