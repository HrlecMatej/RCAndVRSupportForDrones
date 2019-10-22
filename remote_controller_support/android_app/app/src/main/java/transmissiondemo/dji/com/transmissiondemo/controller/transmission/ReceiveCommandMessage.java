package transmissiondemo.dji.com.transmissiondemo.controller.transmission;

import android.util.Log;

import com.MAVLink.Messages.MAVLinkMessage;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.Predicate;

import io.reactivex.Observable;
import io.reactivex.disposables.Disposable;

public abstract class ReceiveCommandMessage<T extends MAVLinkMessage> extends CommandMessage<T> {

    private final long TIMEOUT_SECONDS = 10;

    protected Predicate<T> filterPredicate;

    protected ReceiveCommandMessage(final int msgId, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(msgId, onSuccessCallback, onFailureCallback);
    }

    @Override
    public void execute() {
        subscribeToRC(false);
    }

    @Override
    public void executeIndefinitely() {
        subscribeToRC(true);
    }


    private Observable<MAVLinkMessage> getObservable() {
        return CommandMessageReceiver.getPublishSubject()
            .filter(mavLinkMessage -> {
                if (mavLinkMessage == CommandMessageReceiver.EMPTY_MSG) {
                    return false;
                }
                // Should filter out every message that doesn't belong here
                if (msgId != mavLinkMessage.msgid) {
                    return false;
                }
                return filterPredicate == null ||
                    filterPredicate.test((T) mavLinkMessage);
            })
            .doOnError((error) -> {
                onFailure();
                Log.w("Observable", "Error occurred in Observable: " + error.getMessage());
            }).doOnComplete(() -> {
                Log.w("Observable", "Does this even fire?");
            });
    }

    private Observable<MAVLinkMessage> getObservableWithTimeout() {
        return getObservable()
            .timeout(TIMEOUT_SECONDS, TimeUnit.SECONDS);
    }

    protected abstract void fillCommand(T msg);

    /**
     * Waits for callback from RC. When he receives it, he tries to parse it.
     * If unsuccessful, it was probably meant for another ReceiveCommandMessage.
     *
     * @return
     */
    private void subscribeToRC(final boolean indefinite) {
        final Observable<MAVLinkMessage> observable =
            indefinite ? getObservable() : getObservableWithTimeout();

        final CompletableFuture<T> future = new CompletableFuture<>();

        final Disposable subscribe = observable
            .subscribe(message -> {
                    CommandMessageReceiver.getPublishSubject()
                        .onNext(CommandMessageReceiver.EMPTY_MSG);
                    future.complete((T) message);
                },
                error -> onFailure());

        try {
            final T message = future.get();
            if (!subscribe.isDisposed()) {
                subscribe.dispose();
            }
            fillCommand(message);
            onSuccess();
        } catch (final Exception e) {
            onFailure();
            e.printStackTrace();
        }
    }
}
