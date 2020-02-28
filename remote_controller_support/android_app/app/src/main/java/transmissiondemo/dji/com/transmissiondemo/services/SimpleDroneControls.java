package transmissiondemo.dji.com.transmissiondemo.services;

import transmissiondemo.dji.com.transmissiondemo.MainActivity;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send.DjiActionCommandFlyTrajectorySend;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send.DjiActionCommandHoverSend;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send.DjiActionCommandLandSend;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.actioncommand.send.DjiActionCommandTakeOffSend;
import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.posestamped.send.PoseStampedSetpointSend;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

/**
 * Contains the basic controls for steering the drone
 * All commands are executed async so that don't block the UI thread
 */
public class SimpleDroneControls {

    private final double MOVE_SIZE = 0.1;
    private final double ROTATE_SIZE = 0.1;

    public void moveForward() { moveDrone(MOVE_SIZE, 0, 0, 0); }

    public void moveBackward() { moveDrone(-MOVE_SIZE, 0, 0, 0); }

    public void moveLeft() { moveDrone(0, MOVE_SIZE, 0, 0); }

    public void moveRight() { moveDrone(0, -MOVE_SIZE, 0, 0); }

    public void moveUp() { moveDrone(0, 0, MOVE_SIZE, 0); }

    public void moveDown() { moveDrone(0, 0, -MOVE_SIZE, 0); }

    public void rotateRight() { moveDrone(0, 0, 0, ROTATE_SIZE); }

    public void rotateLeft() { moveDrone(0, 0, 0, -ROTATE_SIZE); }

    public void takeOff() { new DjiActionCommandTakeOffSend(MainActivity.DJI_DRONE_SYSID).executeAsync(); }

    public void flyTrajectory() { new DjiActionCommandFlyTrajectorySend(MainActivity.DJI_DRONE_SYSID).executeAsync(); }

    public void land() { new DjiActionCommandLandSend(MainActivity.DJI_DRONE_SYSID).executeAsync(); }

    public void hover() { new DjiActionCommandHoverSend(MainActivity.DJI_DRONE_SYSID).executeAsync(); }

    private void moveDrone(final double x, final double y, final double z, final double yaw) {
        final double[] q = Utilities.fromEulerAngles(0, 0, yaw);

        new PoseStampedSetpointSend(
            MainActivity.DJI_DRONE_SYSID,
            new double[]{x, y, z},
            q
        ).executeAsync();
    }
}
