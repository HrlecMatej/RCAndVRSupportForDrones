package transmissiondemo.dji.com.transmissiondemo.controller.transmission;

import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;

import java.util.concurrent.CompletableFuture;

import transmissiondemo.dji.com.transmissiondemo.controller.wrapper.MAVLinkStatsWrapper;
import transmissiondemo.dji.com.transmissiondemo.utilities.DjiThreadPool;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

/**
 * The main class that help pack-up messages.
 * It mostly handles sync and async execution on threads.
 * Further logic is further divided into Receive and Send classes.
 * @param <T>
 */
public abstract class DjiMessage<T extends MAVLinkMessage> {

    final int msgId;

    private boolean continueRepeatingExecute = true;

    /**
     * Callback that is called, if the command executes successfully
     */
    protected Runnable onSuccessCallback;
    /**
     * Callback that is called, if the command fails
     */
    protected Runnable onFailureCallback;

    DjiMessage(final int msgId, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        this.msgId = msgId;
        this.onSuccessCallback = onSuccessCallback;
        this.onFailureCallback = onFailureCallback;
    }

    /**
     * If a message has been successfully sent/received.
     * Each command has to implement, how to continue.
     */
    protected void onSuccess() {
        if (onSuccessCallback != null) {
            onSuccessCallback.run();
        }
    }

    public void setOnSuccessCallback(final Runnable onSuccessCallback) {
        this.onSuccessCallback = onSuccessCallback;
    }

    /**
     * If there was a failure on message sending/receiving.
     * Each command has to implement, how to continue.
     */
    protected void onFailure() {
        if (onFailureCallback != null) {
            onFailureCallback.run();
        }
        Utilities.showToast("Command " + getClass().getSimpleName() + " failed!");
    }

    public void setOnFailureCallback(final Runnable onFailureCallback) {
        this.onFailureCallback = onFailureCallback;
    }

    /**
     * A thread blocking action, which tries to resolve this and all follow-up commands.
     * Will timeout after a while
     */
    public abstract void execute();

    /**
     * A thread blocking action, similar to execute, although it tries to fulfil itself indefinitely.
     */
    public abstract void executeIndefinitely();

    /**
     * A non-blocking action, which tries to resolve this and all follow-up commands.
     * Will timeout after a while.
     */
    public void executeAsync() {
        DjiThreadPool.post(() ->
            execute()
        );
    }

    public CompletableFuture<Void> executeAsyncWithFuture() {
        final CompletableFuture<Void> future = new CompletableFuture();
        DjiThreadPool.post(() -> {
            execute();
            future.complete(null);
        });
        return future;
    }

    /**
     * A non-blocking action, which tries to resolve this and all follow-up commands.
     *
     * @return
     */
    public void executeIndefinitelyAsync() {
        new Thread(() ->
            executeIndefinitely()
        ).start();
    }

    public CompletableFuture<Void> executeIndefinitelyAsyncWithFuture() {
        final CompletableFuture<Void> future = new CompletableFuture();
        new Thread(() -> {
            executeIndefinitely();
            future.complete(null);
        }).start();
        return future;
    }

    /**
     * A non-blocking action, which tries to resolve this and all follow-up commands.
     * It will repeat itself after each execution
     */
    public void repeatExecuteIndefinitelyAsync() {
        new Thread(() -> {
            while (continueRepeatingExecute) {
                executeIndefinitely();
            }
        }).start();
    }

    public void cancelRepeatExecuteIndefinitelyAsync() {
        continueRepeatingExecute = false;
    }


}
