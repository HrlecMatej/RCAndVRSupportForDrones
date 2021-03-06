/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This class was automatically generated by the
 * java mavlink generator tool. It should not be modified by hand.
 */

// MESSAGE ROS_NAV_SAT_FIX PACKING
package com.MAVLink.dji_icg;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.Messages.MAVLinkPayload;
        
/**
* Simple commands for the drone.
*/
public class msg_ros_nav_sat_fix extends MAVLinkMessage{

    public static final int MAVLINK_MSG_ID_ROS_NAV_SAT_FIX = 50006;
    public static final int MAVLINK_MSG_ID_ROS_NAV_SAT_FIX_CRC = 179;
    public static final int MAVLINK_MSG_LENGTH = 36;
    private static final long serialVersionUID = MAVLINK_MSG_ID_ROS_NAV_SAT_FIX;


      
    /**
    * Latitude
    */
    public double latitude;
      
    /**
    * Longitude
    */
    public double longitude;
      
    /**
    * Altitude
    */
    public double altitude;
      
    /**
    * Timestamp - Seconds since epoch
    */
    public long sec;
      
    /**
    * Timestamp since epoch = sec + nsec * 10^-9
    */
    public long nsec;
      
    /**
    * See ROS_NAV_SAT_SERVICE enum
    */
    public int service;
      
    /**
    * System ID
    */
    public short target_system;
      
    /**
    * See ROS_NAV_SAT_STATUS enum
    */
    public byte status;
    

    /**
    * Generates the payload for a mavlink message for a message of this type
    * @return
    */
    public MAVLinkPacket pack(){
        MAVLinkPacket packet = new MAVLinkPacket(MAVLINK_MSG_LENGTH);
        packet.sysid = 255;
        packet.compid = 190;
        packet.msgid = MAVLINK_MSG_ID_ROS_NAV_SAT_FIX;
        packet.crc_extra = MAVLINK_MSG_ID_ROS_NAV_SAT_FIX_CRC;
              
        packet.payload.putDouble(latitude);
              
        packet.payload.putDouble(longitude);
              
        packet.payload.putDouble(altitude);
              
        packet.payload.putUnsignedInt(sec);
              
        packet.payload.putUnsignedInt(nsec);
              
        packet.payload.putUnsignedShort(service);
              
        packet.payload.putUnsignedByte(target_system);
              
        packet.payload.putByte(status);
        
        return packet;
    }

    /**
    * Decode a ros_nav_sat_fix message into this class fields
    *
    * @param payload The message to decode
    */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
              
        this.latitude = payload.getDouble();
              
        this.longitude = payload.getDouble();
              
        this.altitude = payload.getDouble();
              
        this.sec = payload.getUnsignedInt();
              
        this.nsec = payload.getUnsignedInt();
              
        this.service = payload.getUnsignedShort();
              
        this.target_system = payload.getUnsignedByte();
              
        this.status = payload.getByte();
        
    }

    /**
    * Constructor for a new message, just initializes the msgid
    */
    public msg_ros_nav_sat_fix(){
        msgid = MAVLINK_MSG_ID_ROS_NAV_SAT_FIX;
    }

    /**
    * Constructor for a new message, initializes the message with the payload
    * from a mavlink packet
    *
    */
    public msg_ros_nav_sat_fix(MAVLinkPacket mavLinkPacket){
        this.sysid = mavLinkPacket.sysid;
        this.compid = mavLinkPacket.compid;
        this.msgid = MAVLINK_MSG_ID_ROS_NAV_SAT_FIX;
        unpack(mavLinkPacket.payload);
    }

                    
    /**
    * Returns a string with the MSG name and data
    */
    public String toString(){
        return "MAVLINK_MSG_ID_ROS_NAV_SAT_FIX - sysid:"+sysid+" compid:"+compid+" latitude:"+latitude+" longitude:"+longitude+" altitude:"+altitude+" sec:"+sec+" nsec:"+nsec+" service:"+service+" target_system:"+target_system+" status:"+status+"";
    }
}
        