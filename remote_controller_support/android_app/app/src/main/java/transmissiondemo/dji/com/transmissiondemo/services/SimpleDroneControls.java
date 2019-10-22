package transmissiondemo.dji.com.transmissiondemo.services;

import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send.DjiActionCommandFlyTrajectorySend;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send.DjiActionCommandHoverSend;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send.DjiActionCommandLandSend;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send.DjiActionCommandTakeOffSend;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.posestamped.send.PoseStampedSetpointSend;

/**
 * Contains the basic controls for stearing the drone
 * All commands are executed async so that don't block the UI thread
 */
public class SimpleDroneControls {

    private final double MOVE_SIZE = 0.1;
    private final double ROTATE_SIZE = 0.1;

    public void moveForward() {
        moveDrone(MOVE_SIZE, 0, 0, 0, 0, 0);
    }

    public void moveBackward() {
        moveDrone(-MOVE_SIZE, 0, 0, 0, 0, 0);

    }

    public void moveLeft() {
        moveDrone(0, MOVE_SIZE, 0, 0, 0, 0);

    }

    public void moveRight() {
        moveDrone(0, -MOVE_SIZE, 0, 0, 0, 0);
    }

    public void moveUp() {
        moveDrone(0, 0, MOVE_SIZE, 0, 0, 0);
    }

    public void moveDown() {
        moveDrone(0, 0, -MOVE_SIZE, 0, 0, 0);
    }

    public void rotateRight() {
        moveDrone(0, 0, 0, 0, 0, ROTATE_SIZE);
    }

    public void rotateLeft() {
        moveDrone(0, 0, 0, 0, 0, -ROTATE_SIZE);
    }

    private void moveDrone(final double x, final double y, final double z, final double roll, final double pitch, final double yaw) {
        final double[] q = fromEulerAngles(roll, pitch, yaw);

        final PoseStampedSetpointSend command = new PoseStampedSetpointSend(
            MainActivity.DJI_DRONE_SYSID,
            new double[]{x, y, z},
            q
        );
        command.executeAsync();
    }

    public void takeOff() {
        final DjiActionCommandTakeOffSend command = new DjiActionCommandTakeOffSend(MainActivity.DJI_DRONE_SYSID);
        command.executeAsync();
    }

    public void flyTrajectory() {
        final DjiActionCommandFlyTrajectorySend command = new DjiActionCommandFlyTrajectorySend(MainActivity.DJI_DRONE_SYSID);
        command.executeAsync();
    }

    public void land() {
        final DjiActionCommandLandSend command = new DjiActionCommandLandSend(MainActivity.DJI_DRONE_SYSID);
        command.executeAsync();
    }

    public void hover() {
        final DjiActionCommandHoverSend command = new DjiActionCommandHoverSend(MainActivity.DJI_DRONE_SYSID);
        command.executeAsync();
    }

    private double[] fromEulerAngles(final double roll, final double pitch, final double yaw) {
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
}
