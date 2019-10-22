package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send;

import com.MAVLink.enums.DJI_ACTION;

public class DjiActionCommandLandSend extends DjiActionCommandSend {

    public DjiActionCommandLandSend(final short targetSystem) {
        this(targetSystem, null, null);
    }

    protected DjiActionCommandLandSend(final short targetSystem, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(targetSystem, DJI_ACTION.DJI_ACTION_LAND, onSuccessCallback, onFailureCallback);
    }
}
