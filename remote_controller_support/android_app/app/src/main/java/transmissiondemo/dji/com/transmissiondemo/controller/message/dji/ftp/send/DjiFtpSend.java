package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.ftp.send;

import com.MAVLink.dji_icg.msg_dji_ftp;

import transmissiondemo.dji.com.transmissiondemo.controller.transmission.SendDjiMessage;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

public class DjiFtpSend extends SendDjiMessage<msg_dji_ftp> {

    private final short targetSystem;
    private final int sequence;
    private final byte[] payload;
    private final short fileId;

    public DjiFtpSend(final short targetSystem, final int sequence, final byte[] payload, final short fileId) {
        this(targetSystem, sequence, payload, fileId, null, null);
    }

    public DjiFtpSend(final short targetSystem, final int sequence, final byte[] payload, final short fileId, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(msg_dji_ftp.MAVLINK_MSG_ID_DJI_FTP, onSuccessCallback, onFailureCallback);
        this.targetSystem = targetSystem;
        this.sequence = sequence;
        this.payload = payload;
        this.fileId = fileId;
    }

    @Override
    protected msg_dji_ftp fillMessage() {
        final msg_dji_ftp msg = new msg_dji_ftp();
        msg.target_system = targetSystem;
        msg.sequence = sequence;
        msg.payload = Utilities.byteArrayToShortArray(payload);
        msg.file_id = fileId;
        return msg;
    }
}
