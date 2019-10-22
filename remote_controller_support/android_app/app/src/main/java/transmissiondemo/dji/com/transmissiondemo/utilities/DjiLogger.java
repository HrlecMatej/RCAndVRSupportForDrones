package transmissiondemo.dji.com.transmissiondemo.utilities;

import android.widget.ProgressBar;
import android.widget.TextView;

import java.text.SimpleDateFormat;
import java.util.Date;

import transmissiondemo.dji.com.transmissiondemo.MainActivity;

public class DjiLogger {
    private static DjiLogger instance;

    private final TextView textLogger;
    SimpleDateFormat simpleDateFormat;
    ProgressBar progressBar;

    private DjiLogger() {
        textLogger = MainActivity.getInstance().textLogger;
        simpleDateFormat = new SimpleDateFormat("HH:mm:ss.SSS");
        progressBar = MainActivity.getInstance().progressBarFile;
    }

    public static DjiLogger getInstance() {
        if (instance == null) {
            synchronized (DjiLogger.class) {
                if (instance == null) {
                    instance = new DjiLogger();
                }
            }
        }
        return instance;
    }

    private String getCurrentTimeStamp() {
        return simpleDateFormat.format(new Date());
    }

    public void logMessage(final String message) {
        if (textLogger != null) {
            Utilities.runOnUi(() ->
                textLogger.setText(getCurrentTimeStamp() + ": " + message)
            );

        }
    }

    public void updateFileUploadProgress(final int progress) {
        if (progressBar != null) {
            Utilities.runOnUi(
                () -> progressBar.setProgress(progress)
            );
        }
    }
}
