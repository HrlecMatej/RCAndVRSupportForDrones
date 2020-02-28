package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.ftp.receive;

import com.MAVLink.dji_icg.msg_dji_ftp;
import com.MAVLink.dji_icg.msg_dji_ftp_request_resend;

import java.util.Arrays;

import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.ftp.send.DjiFtpSend;
import transmissiondemo.dji.com.transmissiondemo.controller.transmission.ReceiveDjiMessage;

public class DjiFtpRequestResendReceive extends ReceiveDjiMessage<msg_dji_ftp_request_resend> {

    private final short targetDroneSystem;
    private final short fileId;
    private final byte[] data;

    private int sequenceStart;
    private int sequenceEnd;

    public DjiFtpRequestResendReceive(final short targetDroneSystem, final short fileId, final byte[] data) {
        this(targetDroneSystem, fileId, data, null, null);
    }

    public DjiFtpRequestResendReceive(final short targetDroneSystem, final short fileId, final byte[] data, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(msg_dji_ftp_request_resend.MAVLINK_MSG_ID_DJI_FTP_REQUEST_RESEND, onSuccessCallback, onFailureCallback);
        this.targetDroneSystem = targetDroneSystem;
        this.fileId = fileId;
        this.data = data;
        filterPredicate = msg ->
            fileId == msg.file_id &&
                MainActivity.DJI_GROUND_CONTROL_ID == msg.target_system;
    }


    @Override
    protected void fillMessage(final msg_dji_ftp_request_resend msg) {
        sequenceStart = msg.sequence_start;
        sequenceEnd = msg.sequence_end;
    }

    @Override
    protected void onSuccess() {
        if (sequenceStart == 0xffff && sequenceEnd == 0xffff) {
            return;
        }
        final int payloadLength = new msg_dji_ftp().payload.length;
        final int byteStart = sequenceStart * payloadLength, byteEnd = sequenceEnd * payloadLength;

        int sequenceCounter = 0;
        for (int i = byteStart; i <= byteEnd; i = i + payloadLength, sequenceCounter++) {
            final byte[] filePart = Arrays.copyOfRange(data, i, i + payloadLength);
            final DjiFtpSend ftpSend = new DjiFtpSend(
                targetDroneSystem,
                sequenceCounter,
                filePart,
                fileId);
            ftpSend.execute();
        }

        final DjiFtpRequestResendReceive resendReceive = new DjiFtpRequestResendReceive(
            targetDroneSystem,
            fileId,
            data);
        resendReceive.execute();
    }
}
