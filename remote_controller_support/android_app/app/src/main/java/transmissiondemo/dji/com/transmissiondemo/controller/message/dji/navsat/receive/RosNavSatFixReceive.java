package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.navsat.receive;

import com.MAVLink.dji_icg.msg_ros_nav_sat_fix;

import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.controller.transmission.ReceiveCommandMessage;

public class RosNavSatFixReceive extends ReceiveCommandMessage<msg_ros_nav_sat_fix> {

    public long sec;
    public long nsec;

    public double latitude;
    public double longitude;
    public double altitude;

    public byte status;
    public int service;

    public RosNavSatFixReceive() {
        this(null, null);
    }

    public RosNavSatFixReceive(final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(msg_ros_nav_sat_fix.MAVLINK_MSG_ID_ROS_NAV_SAT_FIX, onSuccessCallback, onFailureCallback);

        filterPredicate = msg -> MainActivity.DJI_GROUND_CONTROL_ID == msg.target_system;
    }

    @Override
    protected void fillCommand(final msg_ros_nav_sat_fix msg) {
        sec = msg.sec;
        nsec = msg.nsec;

        latitude = msg.latitude;
        longitude = msg.longitude;
        altitude = msg.altitude;

        status = msg.status;
        service = msg.service;
    }
}
