package transmissiondemo.dji.com.transmissiondemo.services;

import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.navsat.receive.RosNavSatFixReceive;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.odometry.receive.RosOdometryReceive;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

import static java.lang.Math.abs;
import static java.lang.Math.asin;
import static java.lang.Math.atan2;
import static java.lang.Math.signum;

public class DroneStatus {

    public DroneStatus() {
        listenForRosNavSatFix();
        listenForRosOdometry();
    }

    private void listenForRosNavSatFix() {
        final RosNavSatFixReceive command = new RosNavSatFixReceive();
        command.setOnSuccessCallback(displayRosNavSatFix(command));
        command.repeatExecuteIndefinitelyAsync();
    }

    private Runnable displayRosNavSatFix(final RosNavSatFixReceive command) {
        return () -> Utilities.runOnUi(() -> {
            final MainActivity activity = MainActivity.getInstance();
            activity.textGpsLatitude.setText(String.format("%.9f", command.latitude));
            activity.textGpsLongitude.setText(String.format("%.9f", command.longitude));
            activity.textGpsAltitude.setText(String.format("%.3f", command.altitude));
        });
    }

    private void listenForRosOdometry() {
        final RosOdometryReceive command = new RosOdometryReceive();
        command.setOnSuccessCallback(displayRosOdometry(command));
        command.repeatExecuteIndefinitelyAsync();
    }

    private Runnable displayRosOdometry(final RosOdometryReceive command) {
        return () -> Utilities.runOnUi(() -> {
            final MainActivity activity = MainActivity.getInstance();
            activity.textPositionX.setText(String.format("%.2f", command.position[0]));
            activity.textPositionY.setText(String.format("%.2f", command.position[1]));
            activity.textPositionZ.setText(String.format("%.2f", command.position[2]));

            final double[] orientation = toEulerAngles(command.orientation);
            activity.textOrientationRoll.setText(String.format("%.2f", orientation[0]));
            activity.textOrientationPitch.setText(String.format("%.2f", orientation[1]));
            activity.textOrientationYaw.setText(String.format("%.2f", orientation[2]));

            activity.textLinearVelocityX.setText(String.format("%.2f", command.linear[0]));
            activity.textLinearVelocityY.setText(String.format("%.2f", command.linear[1]));
            activity.textLinearVelocityZ.setText(String.format("%.2f", command.linear[2]));

            activity.textAngularVelocityX.setText(String.format("%.2f", command.angular[0]));
            activity.textAngularVelocityY.setText(String.format("%.2f", command.angular[1]));
            activity.textAngularVelocityZ.setText(String.format("%.2f", command.angular[2]));
        });
    }

    /**
     * Returns the components of the quaternion if it is represented
     * as standard roll-pitch-yaw Euler angles.
     *
     * @return an array of the form {roll, pitch, yaw}.
     */
    private double[] toEulerAngles(final double[] quaternion) {
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
