package transmissiondemo.dji.com.transmissiondemo;

import android.util.Log;

import java.util.Arrays;

import dji.common.error.DJIError;
import dji.common.util.CommonCallbacks.CompletionCallback;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.flightcontroller.FlightController.OnboardSDKDeviceDataCallback;
import dji.sdk.products.Aircraft;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;


/**
 * Created by sickness on 21.2.2018.
 */

/**
 * This class was used to measure bandwidth
 */
public class TransmitTest {

    final static int DJI_PACKET_SIZE = 100;

    public void sendPing(final MainActivity mainActivity, final int numberToRepeat) {
        final byte[] numberChain = new byte[DJI_PACKET_SIZE];
        Arrays.fill(numberChain, (byte) numberToRepeat);

        Utilities.showToast(mainActivity, "Pinging with: " + numberToRepeat);
        sendData(mainActivity, sendPingCallback(mainActivity), numberChain);
    }

    public void receivePing(final MainActivity mainActivity) {
        receiveData(mainActivity, receivePingCallback());
    }

    protected void receiveData(final MainActivity mainActivity, final OnboardSDKDeviceDataCallback callback) {
        final Aircraft product = (Aircraft) mainActivity.getProduct();
        product.getFlightController().setOnboardSDKDeviceDataCallback(callback);

    }

    protected void sendData(final MainActivity mainActivity, final CompletionCallback callback, final byte[] data) {
        final Aircraft product = (Aircraft) mainActivity.getProduct();

        /**
         * DJI SDK Open Protocol
         * CMD Frame Data
         * 0. byte: CMD Set = 0x02
         * 1. byte: CMD ID = 0x02
         * 2. - 101. byte: CMD Val = whatever
         * I think this is done by the Onboard ROS library
         */
        final FlightController flightController = product.getFlightController();
        flightController.sendDataToOnboardSDKDevice(data, callback);
    }

    private OnboardSDKDeviceDataCallback receivePingCallback() {
        return new OnboardSDKDeviceDataCallback() {
            @Override
            public void onReceive(final byte[] message) {
                Log.i("Ping", Arrays.toString(message));

                // Do a check-up if all the values have the same non-zero values
                if ((message[0] & 0xFF) == 0) {
                    Log.w("Ping", "First byte equals 0");
                    return;
                }
                for (int i = 1; i < message.length; i++) {
                    if (message[0] != message[i]) {
                        Log.w("Ping", "Value: " + (message[i] & 0xFF) + " at: " + i + " is different.");
                        return;
                    }
                }
            }
        };
    }

    private CompletionCallback sendPingCallback(final MainActivity mainActivity) {
        return new CompletionCallback() {
            @Override
            public void onResult(final DJIError pError) {
                if (pError == null) {
                    Utilities.showToast(mainActivity, "The ping was SUCCESSFUL :D !");
                    Log.i("Ping", "Successful ping");
                } else {
                    Utilities.showToast(mainActivity, "Ping error " + pError.getDescription());
                    Log.e("DJIErrorTag", pError.getDescription());
                }
            }
        };
    }
}
