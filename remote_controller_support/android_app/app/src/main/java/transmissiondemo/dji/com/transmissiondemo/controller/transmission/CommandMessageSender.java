package transmissiondemo.dji.com.transmissiondemo.controller.transmission;

import android.util.Log;
import android.util.Pair;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.LinkedBlockingQueue;

import dji.common.error.DJIError;
import dji.common.util.CommonCallbacks.CompletionCallback;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

public class CommandMessageSender {

    private static CommandMessageSender instance;
    private FlightController flightController;

    private final BlockingQueue<Pair<SendCommandMessage, CompletableFuture<Void>>> queue = new LinkedBlockingQueue<>();

    private CommandMessageSender() {
        final Aircraft product = MainActivity.getAircraft();
        try {
            flightController = product.getFlightController();
        } catch (final Exception ex) {
            Log.e("Aircraft", "Could not find the Aircraft or the RC");
            Utilities.showToast("Could not find the Aircraft or the RC");
            return;
        }
        startSendingToRC();
    }

    public static CommandMessageSender getInstance() {
        if (instance == null) {
            synchronized (CommandMessageSender.class) {
                if (instance == null) {
                    instance = new CommandMessageSender();
                }
            }
        }
        return instance;
    }

    /**
     * Adds the command to the queue and returns the future, which will resolve itself at one point.
     *
     * @param command
     * @return
     */
    public CompletableFuture<Void> addToQueue(final SendCommandMessage command) {
        final CompletableFuture<Void> future = new CompletableFuture<>();
        final Pair<SendCommandMessage, CompletableFuture<Void>> pair = new Pair<>(command, future);
        queue.add(pair);
        return future;
    }

    /**
     * Takes commands from the queue and sends them. If there is none, it waits.
     */
    private void startSendingToRC() {
        new Thread(() -> {
            while (true) {
                try {
                    final Pair<SendCommandMessage, CompletableFuture<Void>> pair = queue.take();
                    sendBytesToRC(pair);
                } catch (final InterruptedException e) {
                    Utilities.showToast("Something broke with Sender queue: " + e.getMessage());
                    e.printStackTrace();
                }
            }
        }).start();
    }

    private byte[][] splitBytes(final byte[] array, final int chunkSize) {
        final int numOfChunks = (array.length + chunkSize - 1) / chunkSize;
        final byte[][] output = new byte[numOfChunks][];

        for (int i = 0; i < numOfChunks; ++i) {
            final int start = i * chunkSize;
            final int length = Math.min(array.length - start, chunkSize);

            final byte[] temp = new byte[length];
            System.arraycopy(array, start, temp, 0, length);
            output[i] = temp;
        }

        return output;
    }

    /**
     * Sends the command and waits until that resolves.
     *
     * @param pair
     */
    private void sendBytesToRC(final Pair<SendCommandMessage, CompletableFuture<Void>> pair) {
        final SendCommandMessage command = pair.first;
        final byte[] packetBytes = command.pack();
        final CompletableFuture<Void> future = pair.second;
        final byte[][] splitPacket = splitBytes(packetBytes, 100);

        try {
            for (final byte[] bytes : splitPacket) {
                final CompletableFuture<Void> chunkFuture = new CompletableFuture();
                flightController.sendDataToOnboardSDKDevice(
                    bytes, sendCompletionCallback(chunkFuture));
                chunkFuture.get();
            }
            future.complete(null);
        } catch (final Exception e) {
            Utilities.showToast("Send mavlink future error: " + e.getMessage());
        }
    }

    private CompletionCallback sendCompletionCallback(final CompletableFuture<Void> future) {
        return (final DJIError pError) -> {
            if (pError == null) {
                //Log.i("Mavlink", "Successful Mavlink message");
                future.complete(null);
            } else {
                Utilities.showToast("Mavlink message error " + pError.getDescription());
                Log.e("DJIErrorTag", pError.getDescription());
                future.completeExceptionally(new Throwable(pError.getDescription()));
            }
        };
    }
}
