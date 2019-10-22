/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This class was automatically generated by the
 * java mavlink generator tool. It should not be modified by hand.
 */

// MESSAGE CAMERA_CAPTURE_STATUS PACKING
package com.MAVLink.common;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.Messages.MAVLinkPayload;
        
/**
* Information about the status of a capture
*/
public class msg_camera_capture_status extends MAVLinkMessage{

    public static final int MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS = 262;
    public static final int MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC = 12;
    public static final int MAVLINK_MSG_LENGTH = 18;
    private static final long serialVersionUID = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;


      
    /**
    * Timestamp (milliseconds since system boot)
    */
    public long time_boot_ms;
      
    /**
    * Image capture interval in seconds
    */
    public float image_interval;
      
    /**
    * Time in milliseconds since recording started
    */
    public long recording_time_ms;
      
    /**
    * Available storage capacity in MiB
    */
    public float available_capacity;
      
    /**
    * Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
    */
    public short image_status;
      
    /**
    * Current status of video capturing (0: idle, 1: capture in progress)
    */
    public short video_status;
    

    /**
    * Generates the payload for a mavlink message for a message of this type
    * @return
    */
    public MAVLinkPacket pack(){
        MAVLinkPacket packet = new MAVLinkPacket(MAVLINK_MSG_LENGTH);
        packet.sysid = 255;
        packet.compid = 190;
        packet.msgid = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
        packet.crc_extra = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC;
              
        packet.payload.putUnsignedInt(time_boot_ms);
              
        packet.payload.putFloat(image_interval);
              
        packet.payload.putUnsignedInt(recording_time_ms);
              
        packet.payload.putFloat(available_capacity);
              
        packet.payload.putUnsignedByte(image_status);
              
        packet.payload.putUnsignedByte(video_status);
        
        return packet;
    }

    /**
    * Decode a camera_capture_status message into this class fields
    *
    * @param payload The message to decode
    */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
              
        this.time_boot_ms = payload.getUnsignedInt();
              
        this.image_interval = payload.getFloat();
              
        this.recording_time_ms = payload.getUnsignedInt();
              
        this.available_capacity = payload.getFloat();
              
        this.image_status = payload.getUnsignedByte();
              
        this.video_status = payload.getUnsignedByte();
        
    }

    /**
    * Constructor for a new message, just initializes the msgid
    */
    public msg_camera_capture_status(){
        msgid = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
    }

    /**
    * Constructor for a new message, initializes the message with the payload
    * from a mavlink packet
    *
    */
    public msg_camera_capture_status(MAVLinkPacket mavLinkPacket){
        this.sysid = mavLinkPacket.sysid;
        this.compid = mavLinkPacket.compid;
        this.msgid = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
        unpack(mavLinkPacket.payload);
    }

                
    /**
    * Returns a string with the MSG name and data
    */
    public String toString(){
        return "MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS - sysid:"+sysid+" compid:"+compid+" time_boot_ms:"+time_boot_ms+" image_interval:"+image_interval+" recording_time_ms:"+recording_time_ms+" available_capacity:"+available_capacity+" image_status:"+image_status+" video_status:"+video_status+"";
    }
}
        