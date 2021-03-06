package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send;

import com.MAVLink.enums.DJI_ACTION;

public class DjiActionCommandFlyTrajectorySend extends DjiActionCommandSend {

    public DjiActionCommandFlyTrajectorySend(final short targetSystem) {
        this(targetSystem, null, null);
    }

    public DjiActionCommandFlyTrajectorySend(final short targetSystem, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(targetSystem, DJI_ACTION.DJI_ACTION_FLY_TRAJECTORY, onSuccessCallback, onFailureCallback);
    }
}
