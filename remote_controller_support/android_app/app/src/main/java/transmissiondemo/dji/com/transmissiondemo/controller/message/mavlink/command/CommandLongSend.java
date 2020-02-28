package transmissiondemo.dji.com.transmissiondemo.controller.message.mavlink.command;

import com.MAVLink.common.msg_command_long;

import transmissiondemo.dji.com.transmissiondemo.controller.transmission.SendDjiMessage;

public abstract class CommandLongSend extends SendDjiMessage<msg_command_long> {
    private final short targetSystem;
    private final short targetComponent;
    private final short confirmation;
    private final short command;

    private final float param1;
    private final float param2;
    private final float param3;
    private final float param4;
    private final float param5;
    private final float param6;
    private final float param7;

    protected CommandLongSend(final short targetSystem, final short targetComponent, final short confirmation, final short command, final float param1, final float param2, final float param3, final float param4, final float param5, final float param6, final float param7, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(msg_command_long.MAVLINK_MSG_ID_COMMAND_LONG, onSuccessCallback, onFailureCallback);
        this.targetSystem = targetSystem;
        this.targetComponent = targetComponent;
        this.confirmation = confirmation;
        this.command = command;
        this.param1 = param1;
        this.param2 = param2;
        this.param3 = param3;
        this.param4 = param4;
        this.param5 = param5;
        this.param6 = param6;
        this.param7 = param7;
    }

    @Override
    protected msg_command_long fillMessage() {
        final msg_command_long msg = new msg_command_long();
        msg.target_system = targetSystem;
        msg.target_component = targetComponent;
        msg.confirmation = confirmation;
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param3 = param3;
        msg.param4 = param4;
        msg.param5 = param5;
        msg.param6 = param6;
        msg.param7 = param7;
        return msg;
    }
}
