/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This class was automatically generated by the
 * java mavlink generator tool. It should not be modified by hand.
 */

// MESSAGE HIGH_LATENCY2 PACKING
package com.MAVLink.common;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.Messages.MAVLinkPayload;
        
/**
* WIP: Message appropriate for high latency connections like Iridium (version 2)
*/
public class msg_high_latency2 extends MAVLinkMessage{

    public static final int MAVLINK_MSG_ID_HIGH_LATENCY2 = 235;
    public static final int MAVLINK_MSG_ID_HIGH_LATENCY2_CRC = 179;
    public static final int MAVLINK_MSG_LENGTH = 42;
    private static final long serialVersionUID = MAVLINK_MSG_ID_HIGH_LATENCY2;


      
    /**
    * Timestamp (milliseconds since boot or Unix epoch)
    */
    public long timestamp;
      
    /**
    * Latitude, expressed as degrees * 1E7
    */
    public int latitude;
      
    /**
    * Longitude, expressed as degrees * 1E7
    */
    public int longitude;
      
    /**
    * A bitfield for use for autopilot-specific flags (2 byte version).
    */
    public int custom_mode;
      
    /**
    * Altitude above mean sea level
    */
    public short altitude;
      
    /**
    * Altitude setpoint
    */
    public short target_altitude;
      
    /**
    * Distance to target waypoint or position (meters / 10)
    */
    public int target_distance;
      
    /**
    * Current waypoint number
    */
    public int wp_num;
      
    /**
    * Indicates failures as defined in HL_FAILURE_FLAG ENUM. 
    */
    public int failure_flags;
      
    /**
    * Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
    */
    public short type;
      
    /**
    * Autopilot type / class. defined in MAV_AUTOPILOT ENUM
    */
    public short autopilot;
      
    /**
    * Heading (degrees / 2)
    */
    public short heading;
      
    /**
    * Heading setpoint (degrees / 2)
    */
    public short target_heading;
      
    /**
    * Throttle (percentage)
    */
    public short throttle;
      
    /**
    * Airspeed (m/s * 5)
    */
    public short airspeed;
      
    /**
    * Airspeed setpoint (m/s * 5)
    */
    public short airspeed_sp;
      
    /**
    * Groundspeed (m/s * 5)
    */
    public short groundspeed;
      
    /**
    * Windspeed (m/s * 5)
    */
    public short windspeed;
      
    /**
    * Wind heading (deg / 2)
    */
    public short wind_heading;
      
    /**
    * Maximum error horizontal position since last message (m * 10)
    */
    public short eph;
      
    /**
    * Maximum error vertical position since last message (m * 10)
    */
    public short epv;
      
    /**
    * Air temperature (degrees C) from airspeed sensor
    */
    public byte temperature_air;
      
    /**
    * Maximum climb rate magnitude since last message (m/s * 10)
    */
    public byte climb_rate;
      
    /**
    * Battery (percentage, -1 for DNU)
    */
    public byte battery;
      
    /**
    * Field for custom payload.
    */
    public byte custom0;
      
    /**
    * Field for custom payload.
    */
    public byte custom1;
      
    /**
    * Field for custom payload.
    */
    public byte custom2;
    

    /**
    * Generates the payload for a mavlink message for a message of this type
    * @return
    */
    public MAVLinkPacket pack(){
        MAVLinkPacket packet = new MAVLinkPacket(MAVLINK_MSG_LENGTH);
        packet.sysid = 255;
        packet.compid = 190;
        packet.msgid = MAVLINK_MSG_ID_HIGH_LATENCY2;
        packet.crc_extra = MAVLINK_MSG_ID_HIGH_LATENCY2_CRC;
              
        packet.payload.putUnsignedInt(timestamp);
              
        packet.payload.putInt(latitude);
              
        packet.payload.putInt(longitude);
              
        packet.payload.putUnsignedShort(custom_mode);
              
        packet.payload.putShort(altitude);
              
        packet.payload.putShort(target_altitude);
              
        packet.payload.putUnsignedShort(target_distance);
              
        packet.payload.putUnsignedShort(wp_num);
              
        packet.payload.putUnsignedShort(failure_flags);
              
        packet.payload.putUnsignedByte(type);
              
        packet.payload.putUnsignedByte(autopilot);
              
        packet.payload.putUnsignedByte(heading);
              
        packet.payload.putUnsignedByte(target_heading);
              
        packet.payload.putUnsignedByte(throttle);
              
        packet.payload.putUnsignedByte(airspeed);
              
        packet.payload.putUnsignedByte(airspeed_sp);
              
        packet.payload.putUnsignedByte(groundspeed);
              
        packet.payload.putUnsignedByte(windspeed);
              
        packet.payload.putUnsignedByte(wind_heading);
              
        packet.payload.putUnsignedByte(eph);
              
        packet.payload.putUnsignedByte(epv);
              
        packet.payload.putByte(temperature_air);
              
        packet.payload.putByte(climb_rate);
              
        packet.payload.putByte(battery);
              
        packet.payload.putByte(custom0);
              
        packet.payload.putByte(custom1);
              
        packet.payload.putByte(custom2);
        
        return packet;
    }

    /**
    * Decode a high_latency2 message into this class fields
    *
    * @param payload The message to decode
    */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
              
        this.timestamp = payload.getUnsignedInt();
              
        this.latitude = payload.getInt();
              
        this.longitude = payload.getInt();
              
        this.custom_mode = payload.getUnsignedShort();
              
        this.altitude = payload.getShort();
              
        this.target_altitude = payload.getShort();
              
        this.target_distance = payload.getUnsignedShort();
              
        this.wp_num = payload.getUnsignedShort();
              
        this.failure_flags = payload.getUnsignedShort();
              
        this.type = payload.getUnsignedByte();
              
        this.autopilot = payload.getUnsignedByte();
              
        this.heading = payload.getUnsignedByte();
              
        this.target_heading = payload.getUnsignedByte();
              
        this.throttle = payload.getUnsignedByte();
              
        this.airspeed = payload.getUnsignedByte();
              
        this.airspeed_sp = payload.getUnsignedByte();
              
        this.groundspeed = payload.getUnsignedByte();
              
        this.windspeed = payload.getUnsignedByte();
              
        this.wind_heading = payload.getUnsignedByte();
              
        this.eph = payload.getUnsignedByte();
              
        this.epv = payload.getUnsignedByte();
              
        this.temperature_air = payload.getByte();
              
        this.climb_rate = payload.getByte();
              
        this.battery = payload.getByte();
              
        this.custom0 = payload.getByte();
              
        this.custom1 = payload.getByte();
              
        this.custom2 = payload.getByte();
        
    }

    /**
    * Constructor for a new message, just initializes the msgid
    */
    public msg_high_latency2(){
        msgid = MAVLINK_MSG_ID_HIGH_LATENCY2;
    }

    /**
    * Constructor for a new message, initializes the message with the payload
    * from a mavlink packet
    *
    */
    public msg_high_latency2(MAVLinkPacket mavLinkPacket){
        this.sysid = mavLinkPacket.sysid;
        this.compid = mavLinkPacket.compid;
        this.msgid = MAVLINK_MSG_ID_HIGH_LATENCY2;
        unpack(mavLinkPacket.payload);
    }

                                                          
    /**
    * Returns a string with the MSG name and data
    */
    public String toString(){
        return "MAVLINK_MSG_ID_HIGH_LATENCY2 - sysid:"+sysid+" compid:"+compid+" timestamp:"+timestamp+" latitude:"+latitude+" longitude:"+longitude+" custom_mode:"+custom_mode+" altitude:"+altitude+" target_altitude:"+target_altitude+" target_distance:"+target_distance+" wp_num:"+wp_num+" failure_flags:"+failure_flags+" type:"+type+" autopilot:"+autopilot+" heading:"+heading+" target_heading:"+target_heading+" throttle:"+throttle+" airspeed:"+airspeed+" airspeed_sp:"+airspeed_sp+" groundspeed:"+groundspeed+" windspeed:"+windspeed+" wind_heading:"+wind_heading+" eph:"+eph+" epv:"+epv+" temperature_air:"+temperature_air+" climb_rate:"+climb_rate+" battery:"+battery+" custom0:"+custom0+" custom1:"+custom1+" custom2:"+custom2+"";
    }
}
        