package transmissiondemo.dji.com.transmissiondemo;

import transmissiondemo.dji.com.transmissiondemo.controller.message.dji.posestamped.send.PoseStampedSetpointSend;

public class DroneControls {

    /**
     * This defines the north pole singularity cutoff when converting
     * from quaternions to Euler angles.
     */
    public static final double SINGULARITY_NORTH_POLE = 0.49999;

    /**
     * This defines the south pole singularity cutoff when converting
     * from quaternions to Euler angles.
     */
    public static final double SINGULARITY_SOUTH_POLE = -0.49999;

    private final float STEP_SIZE = 0.1f;
    private final float ROTATE_SIZE = 0.1f;

    private float x, y, z;
    private float roll, pitch, yaw;

    public DroneControls(final float x, final float y, final float z, final float roll, final float pitch, final float yaw) {
        setPosition(x, y, z, roll, pitch, yaw);
    }

    public void setPosition(final float x, final float y, final float z, final float roll, final float pitch, final float yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }

    public void moveForward() {
        x += STEP_SIZE;
        moveDrone(() -> x -= STEP_SIZE);
    }

    public void moveBackward() {
        x -= STEP_SIZE;
        moveDrone(() -> x += STEP_SIZE);
    }

    public void moveLeft() {
        y += STEP_SIZE;
        moveDrone(() -> y -= STEP_SIZE);
    }

    public void moveRight() {
        y -= STEP_SIZE;
        moveDrone(() -> y += STEP_SIZE);
    }

    private void moveDrone(final Runnable onFailure) {
        final double[] q = fromEulerAngles(roll, pitch, yaw);

        final PoseStampedSetpointSend command = new PoseStampedSetpointSend(
            MainActivity.DJI_DRONE_SYSID,
            new double[]{x, y, z},
            q,
            null,
            onFailure
        );
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

