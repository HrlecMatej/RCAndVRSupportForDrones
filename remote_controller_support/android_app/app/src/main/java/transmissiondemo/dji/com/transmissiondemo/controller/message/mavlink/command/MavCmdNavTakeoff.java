package transmissiondemo.dji.com.transmissiondemo.controller.message.mavlink.command;

import com.MAVLink.enums.MAV_CMD;

public class MavCmdNavTakeoff extends CommandLongSend {

    private final short targetSystem;
    private final short targetComponent;
    private final short confirmation;
    private static final short command = MAV_CMD.MAV_CMD_NAV_TAKEOFF;

    private final float minimumPitch;
    private final float yawAngle;
    private final float latitude;
    private final float longitude;
    private final float altitude;

    public MavCmdNavTakeoff(final short targetSystem, final short targetComponent, final short confirmation, final float minimumPitch, final float yawAngle, final float latitude, final float longitude, final float altitude) {
        this(targetSystem, targetComponent, confirmation, minimumPitch, yawAngle, latitude, longitude, altitude, null, null);
    }

    public MavCmdNavTakeoff(final short targetSystem, final short targetComponent, final short confirmation, final float minimumPitch, final float yawAngle, final float latitude, final float longitude, final float altitude, final Runnable onSuccessCallback, final Runnable onFailureCallback) {
        super(targetSystem, targetComponent, confirmation, command, minimumPitch, 0, 0, yawAngle, longitude, latitude, altitude, onSuccessCallback, onFailureCallback);
        this.targetSystem = targetSystem;
        this.targetComponent = targetComponent;
        this.confirmation = confirmation;
        this.minimumPitch = minimumPitch;
        this.yawAngle = yawAngle;
        this.latitude = latitude;
        this.longitude = longitude;
        this.altitude = altitude;
    }
}
