package transmissiondemo.dji.com.transmissiondemo.services.tests;

import android.os.AsyncTask;
import android.os.Environment;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.CompletableFuture;

import dji.common.util.CommonCallbacks.CompletionCallback;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

public class SendBandwidthTestAsync extends AsyncTask<Void, Void, Boolean> {

    final int NUMBER_TO_REPEAT = 42;
    final int NUM_OF_REPETITIONS = 100;

    @Override
    protected Boolean doInBackground(final Void... voids) {

        try {
            final File filesDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
            final File file = new File(filesDir, "bandwidth_send_results.txt");
            final FileOutputStream stream = new FileOutputStream(file);

            for (int sizeInBytes = 100; sizeInBytes >= 0; sizeInBytes -= 10) {
                if (sizeInBytes == 0) {
                    // To test for a 1B message, can't send 0B
                    sizeInBytes = 1;
                }

                Utilities.showToast("Sending messages of length: " + sizeInBytes);

                int numOk = 0;

                final long pingStartTime = System.currentTimeMillis();

                for (int j = 0; j < NUM_OF_REPETITIONS; j++) {
                    if (isCancelled()) {
                        return false;
                    }

                    final CompletableFuture future = new CompletableFuture();

                    sendPing(sizeInBytes, future);

                    try {
                        future.get();
                        numOk++;
                    } catch (final Exception e) {
                    }
                }

                final double totalTimeMilisecs = System.currentTimeMillis() - pingStartTime;

                final double bytesPerSecond = (numOk * sizeInBytes) / (totalTimeMilisecs / 1000.0);
                stream.write((sizeInBytes + " ").getBytes());
                stream.write((bytesPerSecond + " ").getBytes());

                stream.write((numOk + " ").getBytes());
                stream.write(((NUM_OF_REPETITIONS - numOk) + "\n").getBytes());
            }

            stream.flush();
            stream.close();

            Utilities.showToast("Send bandwidth test finished!");

        } catch (final FileNotFoundException e) {
            e.printStackTrace();
        } catch (final IOException e) {
            e.printStackTrace();
        }

        return null;
    }


    public void sendPing(final int sizeInBytes, final CompletableFuture future) {
        final byte[] numberChain = new byte[sizeInBytes];
        Arrays.fill(numberChain, (byte) NUMBER_TO_REPEAT);

        sendData(sendPingCallback(future), future, numberChain);
    }

    protected void sendData(final CompletionCallback callback, final CompletableFuture future, final byte[] data) {
        /**
         * DJI SDK Open Protocol
         * CMD Frame Data
         * 0. byte: CMD Set = 0x02
         * 1. byte: CMD ID = 0x02
         * 2. - 101. byte: CMD Val = whatever
         * This is done by the Onboard ROS library
         */
        final Aircraft product = MainActivity.getAircraft();
        if (product == null || product.getFlightController() == null) {
            future.completeExceptionally(new Throwable("No product or flight controller!"));
        }
        final FlightController flightController = product.getFlightController();

        flightController.sendDataToOnboardSDKDevice(data, callback);
    }

    private CompletionCallback sendPingCallback(final CompletableFuture future) {
        return pError -> {
            if (pError == null) {
                future.complete(null);
            } else {
                future.completeExceptionally(new Throwable(pError.getDescription()));
            }
        };
    }
}
