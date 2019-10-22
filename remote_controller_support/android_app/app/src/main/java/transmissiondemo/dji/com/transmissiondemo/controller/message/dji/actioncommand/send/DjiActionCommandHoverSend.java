package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send;

import com.MAVLink.enums.DJI_ACTION;

public class DjiActionCommandHoverSend extends DjiActionCommandSend {

    public DjiActionCommandHoverSend(final short targetSystem) {
        this(targetSystem, null, null);
    }

    protected DjiActionCommandHoverSend(final short targetSystem, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(targetSystem, DJI_ACTION.DJI_ACTION_HOVER, onSuccessCallback, onFailureCallback);
    }
}
