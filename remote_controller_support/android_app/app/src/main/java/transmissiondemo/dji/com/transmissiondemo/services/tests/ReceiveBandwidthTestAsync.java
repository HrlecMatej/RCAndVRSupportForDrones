package transmissiondemo.dji.com.transmissiondemo.services.tests;

import android.os.AsyncTask;
import android.os.Environment;

import java.io.File;
import java.io.FileOutputStream;
import java.util.concurrent.CompletableFuture;

import dji.sdk.flightcontroller.FlightController;
import dji.sdk.flightcontroller.FlightController.OnboardSDKDeviceDataCallback;
import dji.sdk.products.Aircraft;
import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

public class ReceiveBandwidthTestAsync extends AsyncTask<Void, Void, Boolean> {
    @Override
    protected Boolean doInBackground(final Void... voids) {
        for (int loopRate = 10; loopRate <= 100; loopRate += 10) {
            try {
                final File filesDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
                final File file = new File(filesDir, loopRate + "_bandwidth_receive_results.txt");
                final FileOutputStream stream = new FileOutputStream(file);

                final int NUM_OF_REPETITIONS = 100;
                final int MAGIC_STOPPING_NUMBER = 42;

                int currentMsgLength = 0;
                int numOk = 0;
                long pingStartTime = 0;

                while (!isCancelled()) {
                    final CompletableFuture<byte[]> future = new CompletableFuture<>();
                    receiveData(receivePingCallback(future), future);

                    try {
                        final byte[] message = future.get();

                        if (currentMsgLength == 0 && message.length == MAGIC_STOPPING_NUMBER) {
                            // We are still receiving stop command from before
                            continue;
                        }

                        if (currentMsgLength == 0) {
                            // First value to be set
                            currentMsgLength = message.length;
                            pingStartTime = System.currentTimeMillis();

                            Utilities.showToast("First message length: " + currentMsgLength);
                        } else if (message.length != currentMsgLength) {
                            // We start getting messages of a different length
                            final double totalTimeMilisecs = System.currentTimeMillis() - pingStartTime;
                            final double bytesPerSecond = (numOk * currentMsgLength) / (totalTimeMilisecs / 1000.0);

                            stream.write((currentMsgLength + " ").getBytes());
                            stream.write((bytesPerSecond + " ").getBytes());
                            stream.write((numOk + " ").getBytes());
                            stream.write(((NUM_OF_REPETITIONS - numOk) + "\n").getBytes());

                            currentMsgLength = message.length;
                            Utilities.showToast("Received messages of length: " + currentMsgLength);

                            numOk = 0;
                            pingStartTime = System.currentTimeMillis();

                            if (message.length == MAGIC_STOPPING_NUMBER) {
                                // Magic length number for signifying the stopping of the test
                                Utilities.showToast("Receiving bandwidth finished!");
                                break;
                            }
                        }
                        numOk++;
                    } catch (final Exception e) {
                        e.printStackTrace();
                    }
                }

                stream.flush();
                stream.close();
            } catch (final java.io.IOException e) {
                e.printStackTrace();
            }
        }

        return null;
    }


    private CompletableFuture receiveData(final OnboardSDKDeviceDataCallback callback, final CompletableFuture future) {
        final Aircraft product = MainActivity.getAircraft();
        if (product == null || product.getFlightController() == null) {
            future.completeExceptionally(new Throwable("No product or flight controller!"));
        }
        final FlightController flightController = product.getFlightController();
        flightController.setOnboardSDKDeviceDataCallback(callback);
        return future;
    }

    private OnboardSDKDeviceDataCallback receivePingCallback(final CompletableFuture future) {
        return message -> future.complete(message);
    }
}
