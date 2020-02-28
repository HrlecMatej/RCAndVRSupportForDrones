package transmissiondemo.dji.com.transmissiondemo.controller.message.dji.ftp.receive;

import com.MAVLink.dji_icg.msg_dji_ftp;
import com.MAVLink.dji_icg.msg_dji_ftp_request_ack;
import com.MAVLink.enums.DJI_FTP_REQUEST_RESULT;

import java.util.Arrays;

import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.ftp.send.DjiFtpSend;
import transmissiondemo.dji.com.transmissiondemo.controller.transmission.ReceiveDjiMessage;
import transmissiondemo.dji.com.transmissiondemo.utilities.DjiLogger;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

public class DjiFtpRequestAckReceive extends ReceiveDjiMessage<msg_dji_ftp_request_ack> {

    private final short targetDroneSystem;
    private final short fileId;
    private final byte[] data;

    private short result;
    private short progress;

    public DjiFtpRequestAckReceive(final short fileId, final short targetDroneSystem, final byte[] data) {
        this(fileId, targetDroneSystem, data, null, null);
    }

    public DjiFtpRequestAckReceive(final short fileId, final short targetDroneSystem, final byte[] data, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(msg_dji_ftp_request_ack.MAVLINK_MSG_ID_DJI_FTP_REQUEST_ACK, onSuccessCallback, onFailureCallback);
        this.fileId = fileId;
        this.targetDroneSystem = targetDroneSystem;
        this.data = data;
        filterPredicate = msg ->
            fileId == msg.file_id
                && MainActivity.DJI_GROUND_CONTROL_ID == msg.target_system;
    }

    @Override
    protected void fillMessage(final msg_dji_ftp_request_ack msg) {
        result = msg.result;
        progress = msg.progress;
    }

    @Override
    protected void onSuccess() {
        switch (result) {
            case DJI_FTP_REQUEST_RESULT.DJI_FTP_REQUEST_RESULT_ACCEPTED:
                onAccepted();
                break;
            case DJI_FTP_REQUEST_RESULT.DJI_FTP_REQUEST_RESULT_FAILED:
                Utilities.showToast("File creation/access have been denied.");
                onFailure();
                return;
            case DJI_FTP_REQUEST_RESULT.DJI_FTP_REQUEST_RESULT_IN_PROGRESS:
                Utilities.showToast("File transfer in progress: " + progress + " %");
                onFailure();
                return;
        }
        super.onSuccess();
    }

    private void onAccepted() {
        final int payloadLength = new msg_dji_ftp().payload.length;
        final int numOfPackages = (int) Math.ceil((double) data.length / (double) (payloadLength));

        int sequenceCounter = 0;
        for (int i = 0; i < data.length; i = i + payloadLength, sequenceCounter++) {
            final int payloadEnd = Math.min(i + payloadLength, data.length);

            final byte[] filePart = Arrays.copyOfRange(data, i, payloadEnd);
            final DjiFtpSend ftpSend = new DjiFtpSend(
                targetDroneSystem,
                sequenceCounter,
                filePart,
                fileId);
            ftpSend.execute();

            final int percentProgress = (100 * sequenceCounter) / numOfPackages;
            DjiLogger.getInstance().updateFileUploadProgress(percentProgress);
        }

        final DjiFtpRequestResendReceive resendReceive = new DjiFtpRequestResendReceive(
            targetDroneSystem,
            fileId,
            data);
        resendReceive.execute();
        Utilities.showToast("RequestAckReceive completed");
    }
}
