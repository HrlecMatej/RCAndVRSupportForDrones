package transmissiondemo.dji.com.transmissiondemo.services;

import android.net.Uri;
import android.os.AsyncTask;
import android.view.View;
import android.widget.ProgressBar;

import java.io.IOException;
import java.lang.ref.WeakReference;

import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.ftp.send.DjiFtpRequestSend;
import transmissiondemo.dji.com.transmissiondemo.controller.transmission.DjiMessageReceiver;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

public class SendFileAsync extends AsyncTask<Void, Void, Boolean> {

    private final Uri uri;
    private final WeakReference<ProgressBar> progressBar;

    public SendFileAsync(final Uri uri) {
        super();
        this.uri = uri;
        progressBar = new WeakReference<>(MainActivity.getInstance().progressBarFile);
    }

    @Override
    protected Boolean doInBackground(final Void... voids) {
        try {
            Utilities.runOnUi(
                () -> progressBar.get().setVisibility(View.VISIBLE)
            );

            final byte[] bytes = Utilities.readBytesFromUri(uri);
            final String fileName = Utilities.getFileName(uri);

            // Will start up the CommandMessageReceiver
            DjiMessageReceiver.getInstance();

            new DjiFtpRequestSend(
                MainActivity.DJI_DRONE_SYSID,
                fileName,
                bytes
            ).execute();

        } catch (final IOException e) {
            e.printStackTrace();
        }

        Utilities.runOnUi(
            () -> {
                progressBar.get().setVisibility(View.INVISIBLE);
                progressBar.get().setProgress(0);
            }
        );

        return true;
    }
}
