package transmissiondemo.dji.com.transmissiondemo.utilities;

import android.content.Context;
import android.database.Cursor;
import android.net.Uri;
import android.os.Handler;
import android.os.Looper;
import android.provider.OpenableColumns;
import android.util.Pair;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.TimeUnit;

import dji.thirdparty.sanselan.util.IOUtils;
import transmissiondemo.dji.com.transmissiondemo.MainActivity;

import static java.lang.Math.abs;
import static java.lang.Math.asin;
import static java.lang.Math.atan2;
import static java.lang.Math.signum;

/**
 * Created by sickness on 6.3.2018.
 */

public class Utilities {

    private static final Handler handler = new Handler(Looper.getMainLooper());

    private static Toast toast;

    private static void makeText(final Context context, final String sequence, final int duration) {
        if (toast != null) {
            toast.setText(sequence);
        } else {
            toast = Toast.makeText(context, sequence, duration);
        }
        toast.show();
    }

    public static void showNewToast(final String message) {
        runOnUi(() -> Toast.makeText(MainActivity.getInstance(), message, Toast.LENGTH_SHORT));
    }

    public static void showToast(final String message) {
        showToast(MainActivity.getInstance(), message);
    }

    public static void showToast(final Context context, final String message) {
        showToast(context, message, Toast.LENGTH_LONG);
    }

    public static void showToast(final Context context, final String message, final int duration) {
        runOnUi(() -> {
            makeText(context, message, duration);
            //Toast.makeText(context, message, duration).show();
        });
    }

    public static void runOnUi(final Runnable function) {
        //MainActivity.getInstance().runOnUiThread(function);
        //handler.postAtFrontOfQueue(function);
        handler.post(function);
    }

    public static short[] stringToShortArray(final String string) {
        // I presume it's written in ASCII
        final byte[] bytes = string.getBytes(StandardCharsets.US_ASCII);
        return byteArrayToShortArray(bytes);
    }

    public static short[] byteArrayToShortArray(final byte[] bytes) {
        final short[] shorts = new short[bytes.length];
        for (int i = 0; i < bytes.length; i++) {
            shorts[i] = (short) (bytes[i] & 0xff);
        }
        return shorts;
    }

    public static byte[] shortArraytoByteArray(final short[] shorts) {
        final byte[] bytes = new byte[shorts.length];
        for (int i = 0; i < bytes.length; i++) {
            bytes[i] = (byte) shorts[i];
        }
        return bytes;
    }

    public static byte[] readBytesFromUri(final Uri uri) throws IOException {
        final InputStream inputStream =
            MainActivity.getInstance().getContentResolver().openInputStream(uri);
        final byte[] fileBytes = IOUtils.getInputStreamBytes(inputStream);
        return fileBytes;
    }

    public static String getFileName(final Uri uri) {
        String result = null;
        if (uri.getScheme().equals("content")) {
            final Cursor cursor = MainActivity.getInstance().getContentResolver().query(uri, null, null, null, null);
            try {
                if (cursor != null && cursor.moveToFirst()) {
                    result = cursor.getString(cursor.getColumnIndex(OpenableColumns.DISPLAY_NAME));
                }
            } finally {
                cursor.close();
            }
        }
        if (result == null) {
            result = uri.getPath();
            final int cut = result.lastIndexOf('/');
            if (cut != -1) {
                result = result.substring(cut + 1);
            }
        }
        return result;
    }

    /**
     * Returns the current time, where the First is seconds, Second is nanoseconds
     *
     * @return
     */
    public static Pair<Long, Long> getTimestamp() {
        final long currentTime = System.currentTimeMillis();

        // It is rounded down
        final long seconds = TimeUnit.MILLISECONDS.toSeconds(currentTime);
        // Get only the milliseconds from the current second
        final long deltaMilliSecs = currentTime - TimeUnit.SECONDS.toMillis(seconds);
        // Change it to nanoseconds, because that's what we need
        final long nanoSeconds = TimeUnit.MILLISECONDS.toNanos(deltaMilliSecs);
        return new Pair<>(seconds, nanoSeconds);
    }

    public static double[] fromEulerAngles(final double roll, final double pitch, final double yaw) {
        // Apply Euler angle transformations
        final double c1 = Math.cos(yaw / 2.0);
        final double s1 = Math.sin(yaw / 2.0);
        final double c2 = Math.cos(pitch / 2.0);
        final double s2 = Math.sin(pitch / 2.0);
        final double c3 = Math.cos(roll / 2.0);
        final double s3 = Math.sin(roll / 2.0);
        final double c1c2 = c1 * c2;
        final double s1s2 = s1 * s2;

        final double[] q = new double[4];
        // Compute quaternion from components
        q[0] = (c1c2 * c3 - s1s2 * s3);
        q[1] = (c1c2 * s3 + s1s2 * c3);
        q[2] = (s1 * c2 * c3 + c1 * s2 * s3);
        q[3] = (c1 * s2 * c3 - s1 * c2 * s3);
        return q;
    }

    /**
     * Returns the components of the quaternion if it is represented
     * as standard roll-pitch-yaw Euler angles.
     *
     * @return an array of the form {roll, pitch, yaw}.
     */
    public static double[] toEulerAngles(final double[] quaternion) {
        final double x = quaternion[0];
        final double y = quaternion[1];
        final double z = quaternion[2];
        final double w = quaternion[3];
        // roll (x-axis rotation)
        final double sinr = +2.0 * (w * x + y * z);
        final double cosr = +1.0 - 2.0 * (x * x + y * y);
        final double r_roll = atan2(sinr, cosr);

        // pitch (y-axis rotation)
        final double sinp = +2.0 * (w * y - z * x);
        final double r_pitch;
        if (abs(sinp) >= 1) {
            r_pitch = (Math.PI / 2) * signum(sinp); // use 90 degrees if out of range
        } else {
            r_pitch = asin(sinp);
        }

        // yaw (z-axis rotation)
        final double siny = +2.0 * (w * z + x * y);
        final double cosy = +1.0 - 2.0 * (y * y + z * z);
        final double r_yaw = atan2(siny, cosy);

        // Convert to degrees and return
        return new double[]{r_roll * 180 / Math.PI, r_pitch * 180 / Math.PI, r_yaw * 180 / Math.PI};
    }
}
