package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.posestamped.send;

import com.MAVLink.enums.DJI_TOPIC_NAME;

public class PoseStampedSetpointSend extends PoseStampedSend {

    public PoseStampedSetpointSend(final short targetSystem, final double[] pos, final double[] q) {
        this(targetSystem, pos, q, null, null);
    }

    public PoseStampedSetpointSend(final short targetSystem, final double[] position, final double[] orientation, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(targetSystem, DJI_TOPIC_NAME.DJI_TOPIC_NAME_SETPOINT, position, orientation, onSuccessCallback, onFailureCallback);
    }
}
