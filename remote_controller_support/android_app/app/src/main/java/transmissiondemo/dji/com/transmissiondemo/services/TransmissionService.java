package transmissiondemo.dji.com.transmissiondemo.services;

import android.app.Service;
import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;

import java.util.Arrays;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

import dji.common.error.DJIError;
import dji.common.util.CommonCallbacks.CompletionCallback;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

/**
 * Created by sickness on 12.3.2018.
 */

public class TransmissionService extends Service {

    final static int DJI_PACKET_SIZE = 100;

    // Binder given to clients
    private final IBinder transmissionBinder = new TransmissionBinder();
    private MainActivity mainActivity;

    /**
     * Class used for the client Binder.  Because we know this service always
     * runs in the same process as its clients, we don't need to deal with IPC.
     */
    public class TransmissionBinder extends Binder {
        public TransmissionService getService() {
            // Return this instance of LocalService so clients can call public methods
            return TransmissionService.this;
        }
    }

    @Override
    public IBinder onBind(Intent intent) {
        return transmissionBinder;
    }

    public void setMainActivity(MainActivity activity) {
        mainActivity = activity;
    }

    /**
     * method for clients
     */
    public void sendManyPings() {
        int numberToRepeat = 1;

        while (true/*continuePinging*/) {
            Utilities.showToast(mainActivity, "Pinging with: "+numberToRepeat);
            CompletableFuture completePing = sendPing(numberToRepeat);

            try {
                completePing.get();
            } catch (InterruptedException e) {
                e.printStackTrace();
            } catch (ExecutionException e) {
                e.printStackTrace();
            }

            //sendPing(this, numberToRepeat);
            numberToRepeat++;
            if (numberToRepeat == 255) {
                numberToRepeat = 1;
            }
        }
    }

    public CompletableFuture sendPing(int numberToRepeat) {
        byte[] numberChain = new byte[DJI_PACKET_SIZE];
        Arrays.fill(numberChain, (byte)numberToRepeat);

        CompletableFuture future = new CompletableFuture();

        return sendData(sendPingCallback(future), future, numberChain);
    }

    public void receivePing() {
        receiveData(receivePingCallback());
    }

    protected void receiveData(FlightController.OnboardSDKDeviceDataCallback callback) {
        Aircraft product = (Aircraft) mainActivity.getProduct();
        product.getFlightController().setOnboardSDKDeviceDataCallback(callback);

    }

    protected CompletableFuture sendData(CompletionCallback callback, CompletableFuture future, byte[] data) {
        /**
         * DJI SDK Open Protocol
         * CMD Frame Data
         * 0. byte: CMD Set = 0x02
         * 1. byte: CMD ID = 0x02
         * 2. - 101. byte: CMD Val = whatever
         * This is done by the Onboard ROS library
         */

        Aircraft product = (Aircraft) mainActivity.getProduct();
        FlightController flightController = product.getFlightController();

        flightController.sendDataToOnboardSDKDevice(data, callback);
        return future;
    }

    private FlightController.OnboardSDKDeviceDataCallback receivePingCallback() {
        return new FlightController.OnboardSDKDeviceDataCallback() {
            @Override
            public void onReceive(byte[] message) {
                Log.i("Ping", Arrays.toString(message));

                // Do a check-up if all the values have the same non-zero values
                if ((message[0] & 0xFF) == 0) {
                    Log.w("Ping", "First byte equals 0");
                    return;
                }
                for (int i = 1; i < message.length; i++) {
                    if (message[0] != message[i]) {
                        Log.w("Ping", "Value: "+(message[i] & 0xFF)+" at: "+i+" is different.");
                        return;
                    }
                }
            }
        };
    }

    private CompletionCallback sendPingCallback(CompletableFuture future) {
        return new CompletionCallback() {
            @Override
            public void onResult(DJIError pError) {
                if (pError == null) {
                    Utilities.showToast(mainActivity, "The ping was SUCCESSFUL :D !");
                    Log.i("Ping", "Successful ping");
                    future.complete(null);
                }
                else {
                    Utilities.showToast(mainActivity, "Ping error "+pError.getDescription());
                    Log.e("DJIErrorTag", pError.getDescription());
                    future.completeExceptionally(new Throwable(pError.getDescription()));
                }
            }
        };
    }
}
