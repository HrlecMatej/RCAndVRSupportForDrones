package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.odometry.receive;

import com.MAVLink.dji_icg.msg_ros_odometry;

import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.controller.transmission.ReceiveDjiMessage;

public class RosOdometryReceive extends ReceiveDjiMessage<msg_ros_odometry> {

    public long sec;
    public long nsec;

    public double[] position;
    public double[] orientation;

    public double[] linear;
    public double[] angular;

    public RosOdometryReceive() {
        this(null, null);
    }

    public RosOdometryReceive(final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(msg_ros_odometry.MAVLINK_MSG_ID_ROS_ODOMETRY, onSuccessCallback, onFailureCallback);

        filterPredicate = msg -> MainActivity.DJI_GROUND_CONTROL_ID == msg.target_system;
    }

    @Override
    protected void fillMessage(final msg_ros_odometry msg) {
        sec = msg.sec;
        nsec = msg.nsec;

        position = msg.position;
        orientation = msg.orientation;

        linear = msg.linear;
        angular = msg.angular;
    }
}
