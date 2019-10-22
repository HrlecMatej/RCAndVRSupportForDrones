package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send;

import android.util.Pair;

import com.MAVLink.dji_icg.msg_dji_action_command;

import transmissiondemo.dji.com.transmissiondemo.controller.transmission.SendCommandMessage;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

public abstract class DjiActionCommandSend extends SendCommandMessage<msg_dji_action_command> {

    private final short targetSystem;
    private final int action;

    protected DjiActionCommandSend(final short targetSystem, final int action, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(msg_dji_action_command.MAVLINK_MSG_ID_DJI_ACTION_COMMAND, onSuccessCallback, onFailureCallback);
        this.targetSystem = targetSystem;
        this.action = action;
    }

    @Override
    protected msg_dji_action_command fillMessage() {
        final msg_dji_action_command msg = new msg_dji_action_command();
        msg.target_system = targetSystem;

        msg.action = action;

        final Pair<Long, Long> timestamp = Utilities.getTimestamp();
        msg.sec = timestamp.first;
        msg.nsec = timestamp.second;
        return msg;
    }
}
