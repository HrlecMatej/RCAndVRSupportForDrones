package transmissiondemo.dji.com.transmissiondemo.controller.transmission;

import android.util.Log;

import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.Parser;
import com.MAVLink.common.msg_heartbeat;

import dji.sdk.flightcontroller.FlightController;
import dji.sdk.flightcontroller.FlightController.OnboardSDKDeviceDataCallback;
import dji.sdk.products.Aircraft;
import io.reactivex.subjects.BehaviorSubject;
import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

public class CommandMessageReceiver {

    private static CommandMessageReceiver instance;
    private FlightController flightController;
    private Parser parser;

    // Needed to overwrite last received message
    // Will be ignored by the ReceiveCommandMessages
    public static final msg_heartbeat EMPTY_MSG = new msg_heartbeat();

    public BehaviorSubject<MAVLinkMessage> publishSubject;

    private CommandMessageReceiver() {
        final Aircraft product = MainActivity.getAircraft();
        try {
            flightController = product.getFlightController();
        } catch (final Exception ex) {
            Log.e("Aircraft", "Could not find the Aircraft or the RC");
            Utilities.showToast("Could not find the Aircraft or the RC");
            return;
        }
        publishSubject = BehaviorSubject.create();
        publishSubject.onNext(EMPTY_MSG);

        parser = new Parser();

        listenToRC();
    }

    public static CommandMessageReceiver getInstance() {
        if (instance == null) {
            synchronized (CommandMessageReceiver.class) {
                if (instance == null) {
                    instance = new CommandMessageReceiver();
                }
            }
        }
        return instance;
    }

    public static BehaviorSubject<MAVLinkMessage> getPublishSubject() {
        return getInstance().publishSubject;
    }

    private void listenToRC() {
        flightController.setOnboardSDKDeviceDataCallback(receiveCompletionCallback());
    }

    private OnboardSDKDeviceDataCallback receiveCompletionCallback() {
        return (final byte[] message) -> {

            final int previousCrcErrorCount = parser.stats.crcErrorCount;
            MAVLinkPacket packet = null;
            for (final byte b : message) {
                packet = parser.mavlink_parse_char(b);
                if (parser.stats.crcErrorCount != previousCrcErrorCount) {
                    // The Parser increments the crc error count on failure
                    // It also already resets the parsing state
                    Utilities.showToast("Received message parsing failed");
                    break;
                }
            }

            if (packet != null) {
                // TODO: Do something with lostPacketCount
                publishSubject.onNext(packet.unpack());
            }
        };
    }
}
