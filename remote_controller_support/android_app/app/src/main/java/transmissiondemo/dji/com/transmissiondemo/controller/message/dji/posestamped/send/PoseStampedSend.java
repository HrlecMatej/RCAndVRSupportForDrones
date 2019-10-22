package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.posestamped.send;

import android.util.Pair;

import com.MAVLink.dji_icg.msg_dji_pose_stamped;

import transmissiondemo.dji.com.transmissiondemo.controller.transmission.SendCommandMessage;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

public abstract class PoseStampedSend extends SendCommandMessage<msg_dji_pose_stamped> {

    private final short targetSystem;

    private final int topicName;

    private final double[] position;
    private final double[] orientation;

    public PoseStampedSend(final short targetSystem, final int topicName, final double[] position, final double[] orientation) {
        this(targetSystem, topicName, position, orientation, null, null);
    }

    public PoseStampedSend(final short targetSystem, final int topicName, final double[] position, final double[] orientation, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(msg_dji_pose_stamped.MAVLINK_MSG_ID_DJI_POSE_STAMPED, onSuccessCallback, onFailureCallback);
        this.targetSystem = targetSystem;
        this.topicName = topicName;
        this.position = position;
        this.orientation = orientation;
    }

    @Override
    protected msg_dji_pose_stamped fillMessage() {
        final msg_dji_pose_stamped msg = new msg_dji_pose_stamped();
        msg.target_system = targetSystem;

        msg.topic_name = (short) topicName;

        msg.position = position;
        msg.orientation = orientation;

        final Pair<Long, Long> timestamp = Utilities.getTimestamp();
        msg.sec = timestamp.first;
        msg.nsec = timestamp.second;

        return msg;
    }
}
