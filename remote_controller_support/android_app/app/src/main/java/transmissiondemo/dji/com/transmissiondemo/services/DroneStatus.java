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
        final RosNavSatFixReceive navSatFix = new RosNavSatFixReceive();
        navSatFix.setOnSuccessCallback(displayRosNavSatFix(navSatFix));
        navSatFix.repeatExecuteIndefinitelyAsync();
    }

    private Runnable displayRosNavSatFix(final RosNavSatFixReceive navSatFix) {
        return () -> Utilities.runOnUi(() -> {
            final MainActivity activity = MainActivity.getInstance();
            activity.textGpsLatitude.setText(String.format("%.9f", navSatFix.latitude));
            activity.textGpsLongitude.setText(String.format("%.9f", navSatFix.longitude));
            activity.textGpsAltitude.setText(String.format("%.3f", navSatFix.altitude));
        });
    }

    private void listenForRosOdometry() {
        final RosOdometryReceive odometry = new RosOdometryReceive();
        odometry.setOnSuccessCallback(displayRosOdometry(odometry));
        odometry.repeatExecuteIndefinitelyAsync();
    }

    private Runnable displayRosOdometry(final RosOdometryReceive odometry) {
        return () -> Utilities.runOnUi(() -> {
            final MainActivity activity = MainActivity.getInstance();
            activity.textPositionX.setText(String.format("%.2f", odometry.position[0]));
            activity.textPositionY.setText(String.format("%.2f", odometry.position[1]));
            activity.textPositionZ.setText(String.format("%.2f", odometry.position[2]));

            final double[] orientation = Utilities.toEulerAngles(odometry.orientation);
            activity.textOrientationRoll.setText(String.format("%.2f", orientation[0]));
            activity.textOrientationPitch.setText(String.format("%.2f", orientation[1]));
            activity.textOrientationYaw.setText(String.format("%.2f", orientation[2]));

            activity.textLinearVelocityX.setText(String.format("%.2f", odometry.linear[0]));
            activity.textLinearVelocityY.setText(String.format("%.2f", odometry.linear[1]));
            activity.textLinearVelocityZ.setText(String.format("%.2f", odometry.linear[2]));

            activity.textAngularVelocityX.setText(String.format("%.2f", odometry.angular[0]));
            activity.textAngularVelocityY.setText(String.format("%.2f", odometry.angular[1]));
            activity.textAngularVelocityZ.setText(String.format("%.2f", odometry.angular[2]));
        });
    }
}
