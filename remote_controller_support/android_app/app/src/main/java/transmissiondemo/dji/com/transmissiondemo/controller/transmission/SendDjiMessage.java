package transmissiondemo.dji.com.transmissiondemo.controller.transmission;

import android.util.Log;

import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;

import java.util.concurrent.CompletableFuture;

import transmissiondemo.dji.com.transmissiondemo.controller.wrapper.MAVLinkStatsWrapper;
import transmissiondemo.dji.com.transmissiondemo.utilities.DjiLogger;

public abstract class SendDjiMessage<T extends MAVLinkMessage> extends DjiMessage<T> {

    private final int NUM_OF_REPETITIONS = 5;

    protected SendDjiMessage(final int msgId, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(msgId, onSuccessCallback, onFailureCallback);
    }

    @Override
    protected void onSuccess() {
        DjiLogger.getInstance().logMessage(getClass().getSimpleName() + " executed.");
        super.onSuccess();
    }

    @Override
    public void execute() {
        for (int i = 0; i < NUM_OF_REPETITIONS; i++) {
            final CompletableFuture<Void> future = executeSendToRC();
            try {
                future.get();
                onSuccess();
                return;
            } catch (final Exception e) {
                Log.w("SendCommandMessage error", e.toString());
            }
        }
        // If it was not successful after multiple tries
        onFailure();
    }

    @Override
    public void executeIndefinitely() {
        while (true) {
            final CompletableFuture<Void> future = executeSendToRC();
            try {
                future.get();
                onSuccess();
                break;
            } catch (final Exception e) {
                Log.w("SendCommandMessage error", e.toString());
            }
        }
    }

    /**
     * Defines the mapping from our SendDjiMessage to a MavLink one
     * @return
     */
    protected abstract T fillMessage();

    /**
     * Packs and encodes a Mavlink message
     * @return encoded message
     */
    protected byte[] pack() {
        final T msg = fillMessage();
        return encode(msg);
    }

    private byte[] encode(final T msg) {
        final MAVLinkPacket packet = msg.pack();

        final MAVLinkStatsWrapper mavLinkStatsWrapper = MAVLinkStatsWrapper
                .getInstance();
        packet.seq = mavLinkStatsWrapper
                .getSequenceNumber(packet.sysid, packet.compid);
        mavLinkStatsWrapper.mavLinkStats.newPacket(packet);

        final byte[] packetBytes = packet.encodePacket();
        return packetBytes;
    }

    private CompletableFuture<Void> executeSendToRC() {
        return DjiMessageSender.getInstance().addToQueue(this);
    }
}
