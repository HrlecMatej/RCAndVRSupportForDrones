package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.ftp.send;

import com.MAVLink.dji_icg.msg_dji_ftp_request;

import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.ftp.receive.DjiFtpRequestAckReceive;
import transmissiondemo.dji.com.transmissiondemo.controller.transmission.SendCommandMessage;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

public class DjiFtpRequestSend extends SendCommandMessage<msg_dji_ftp_request> {

    // Takes care of file numbering
    private static short fileIdCounter = 0;

    private final short targetSystem;
    private final long fileSize;
    private final String fileName;
    private final short fileId;

    private final byte[] data;

    public DjiFtpRequestSend(final short targetSystem, final String fileName, final byte[] data) {
        this(targetSystem, fileName, data, null, null);
    }

    public DjiFtpRequestSend(final short targetSystem, final String fileName, final byte[] data, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(msg_dji_ftp_request.MAVLINK_MSG_ID_DJI_FTP_REQUEST, onSuccessCallback, onFailureCallback);
        this.targetSystem = targetSystem;
        fileSize = data.length;
        this.fileName = fileName;
        this.data = data;

        fileId = fileIdCounter++;
        if (fileIdCounter >= 256) {
            fileIdCounter = 0;
        }
    }

    @Override
    protected void onSuccess() {
        final DjiFtpRequestAckReceive receiveAck =
            new DjiFtpRequestAckReceive(fileId, targetSystem, data, null, null);
        receiveAck.execute();
        super.onSuccess();
    }

    @Override
    protected msg_dji_ftp_request fillMessage() {
        final msg_dji_ftp_request msg = new msg_dji_ftp_request();
        msg.target_system = targetSystem;
        msg.file_size = fileSize;
        final short[] shorts = Utilities.stringToShortArray(fileName);
        System.arraycopy(shorts, 0, msg.file_name, 0, shorts.length);
        msg.file_name[msg.file_name.length - 1] = 0;
        msg.file_id = fileId;
        return msg;
    }
}
