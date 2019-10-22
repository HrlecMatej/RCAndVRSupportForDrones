package transmissiondemo.dji.com.transmissiondemo.controller.wrapper;

import com.MAVLink.Messages.MAVLinkStats;

/**
 * Singleton, which wraps the Mavlink stats tracker
 */
public class MAVLinkStatsWrapper {

    private static MAVLinkStatsWrapper instance;
    public final MAVLinkStats mavLinkStats;

    private MAVLinkStatsWrapper() {
        mavLinkStats = new MAVLinkStats();
    }

    public static MAVLinkStatsWrapper getInstance() {
        if (instance == null) {
            synchronized (MAVLinkStatsWrapper.class) {
                if (instance == null) {
                    instance = new MAVLinkStatsWrapper();
                }
            }
        }
        return instance;
    }

    /**
     * Takes care of providing the correct sequence number of a new packet
     *
     * @param sysid
     * @param compid
     * @return
     */
    public int getSequenceNumber(final int sysid, final int compid) {
        if (mavLinkStats.systemStats[sysid] == null ||
            mavLinkStats.systemStats[sysid].componentStats[compid] == null) {
            // The component has not been initialized yet, so just set sequence number as 0
            return 0;
        }
        final MAVLinkStats.ComponentStat componentStat = mavLinkStats.systemStats[sysid].componentStats[compid];
        // This number was incremented by the ComponentStats.newPacket() function in advance
        return componentStat.lastPacketSeq;
    }
}
