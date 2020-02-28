package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send;

import com.MAVLink.enums.DJI_ACTION;

public class DjiActionCommandTakeOffSend extends DjiActionCommandSend {

    public DjiActionCommandTakeOffSend(final short targetSystem) {
        this(targetSystem, null, null);
    }

    public DjiActionCommandTakeOffSend(final short targetSystem, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(targetSystem, DJI_ACTION.DJI_ACTION_TAKE_OFF, onSuccessCallback, onFailureCallback);
    }
}
