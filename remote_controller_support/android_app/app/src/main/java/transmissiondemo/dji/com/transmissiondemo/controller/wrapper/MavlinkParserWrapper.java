package transmissiondemo.dji.com.transmissiondemo.controller.wrapper;

import com.MAVLink.Messages.MAVLinkStats;
import com.MAVLink.Parser;

public class MavlinkParserWrapper {

    private static MavlinkParserWrapper instance;
    public final Parser mavlinkParser;

    private MavlinkParserWrapper() {
        mavlinkParser = new Parser();
    }

    public static MavlinkParserWrapper getInstance() {
        if (instance == null) {
            synchronized (MAVLinkStatsWrapper.class) {
                if (instance == null) {
                    instance = new MavlinkParserWrapper();
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
        final MAVLinkStats mavLinkStats = mavlinkParser.stats;
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
