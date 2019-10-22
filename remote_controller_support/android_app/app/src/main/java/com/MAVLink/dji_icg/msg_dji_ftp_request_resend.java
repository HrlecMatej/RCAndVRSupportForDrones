/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This class was automatically generated by the
 * java mavlink generator tool. It should not be modified by hand.
 */

// MESSAGE DJI_FTP_REQUEST_RESEND PACKING
package com.MAVLink.dji_icg;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.Messages.MAVLinkPayload;
        
/**
* After getting the last part or timing out, request individual lost parts of a file. If there are no problems, set sequence numbers to 65,536
*/
public class msg_dji_ftp_request_resend extends MAVLinkMessage{

    public static final int MAVLINK_MSG_ID_DJI_FTP_REQUEST_RESEND = 50003;
    public static final int MAVLINK_MSG_ID_DJI_FTP_REQUEST_RESEND_CRC = 144;
    public static final int MAVLINK_MSG_LENGTH = 6;
    private static final long serialVersionUID = MAVLINK_MSG_ID_DJI_FTP_REQUEST_RESEND;


      
    /**
    * Starting sequence number.
    */
    public int sequence_start;
      
    /**
    * End sequence number.
    */
    public int sequence_end;
      
    /**
    * System which requested the command to be executed.
    */
    public short target_system;
      
    /**
    * Id of the file.
    */
    public short file_id;
    

    /**
    * Generates the payload for a mavlink message for a message of this type
    * @return
    */
    public MAVLinkPacket pack(){
        MAVLinkPacket packet = new MAVLinkPacket(MAVLINK_MSG_LENGTH);
        packet.sysid = 255;
        packet.compid = 190;
        packet.msgid = MAVLINK_MSG_ID_DJI_FTP_REQUEST_RESEND;
        packet.crc_extra = MAVLINK_MSG_ID_DJI_FTP_REQUEST_RESEND_CRC;
              
        packet.payload.putUnsignedShort(sequence_start);
              
        packet.payload.putUnsignedShort(sequence_end);
              
        packet.payload.putUnsignedByte(target_system);
              
        packet.payload.putUnsignedByte(file_id);
        
        return packet;
    }

    /**
    * Decode a dji_ftp_request_resend message into this class fields
    *
    * @param payload The message to decode
    */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
              
        this.sequence_start = payload.getUnsignedShort();
              
        this.sequence_end = payload.getUnsignedShort();
              
        this.target_system = payload.getUnsignedByte();
              
        this.file_id = payload.getUnsignedByte();
        
    }

    /**
    * Constructor for a new message, just initializes the msgid
    */
    public msg_dji_ftp_request_resend(){
        msgid = MAVLINK_MSG_ID_DJI_FTP_REQUEST_RESEND;
    }

    /**
    * Constructor for a new message, initializes the message with the payload
    * from a mavlink packet
    *
    */
    public msg_dji_ftp_request_resend(MAVLinkPacket mavLinkPacket){
        this.sysid = mavLinkPacket.sysid;
        this.compid = mavLinkPacket.compid;
        this.msgid = MAVLINK_MSG_ID_DJI_FTP_REQUEST_RESEND;
        unpack(mavLinkPacket.payload);
    }

            
    /**
    * Returns a string with the MSG name and data
    */
    public String toString(){
        return "MAVLINK_MSG_ID_DJI_FTP_REQUEST_RESEND - sysid:"+sysid+" compid:"+compid+" sequence_start:"+sequence_start+" sequence_end:"+sequence_end+" target_system:"+target_system+" file_id:"+file_id+"";
    }
}
        