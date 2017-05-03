using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MavLink;

namespace MavLink
{

    #region MAVLink Packet

    public class MavlinkPacket
    {
        /// <summary>
        /// The sender's system ID
        /// </summary>
        public int SystemId;

        /// <summary>
        /// The sender's component ID
        /// </summary>
        public int ComponentId;

        /// <summary>
        /// The sequence number received for this packet
        /// </summary>
        public byte SequenceNumber;

        /// <summary>
        /// Time of receipt
        /// </summary>
        //public DateTime TimeStamp;

        /// <summary>
        /// Object which is the mavlink message
        /// </summary>
        public MavlinkMessage Message;
    }

    #endregion

    #region CRC Static Class

    static class Mavlink_Crc
    {
        #region Fields and Const

        const UInt16 X25_INIT_CRC = 0xffff;

        #endregion

        #region Static Methods

        public static UInt16 CrcAccumulate(byte b, UInt16 crc)
        {
            unchecked
            {
                byte ch = (byte)(b ^ (byte)(crc & 0x00ff));
                ch = (byte)(ch ^ (ch << 4));
                return (UInt16)((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4));
            }
        }


        // For a "message" of length bytes contained in the byte array
        // pointed to by buffer, calculate the CRC
        public static UInt16 Calculate(byte[] buffer, UInt16 start, UInt16 length)
        {
            UInt16 crcTmp = X25_INIT_CRC;

            for (int i = start; i < start + length; i++)
                crcTmp = CrcAccumulate(buffer[i], crcTmp);

            return crcTmp;
        }

        #endregion
    }

    #endregion

    #region Mission Item Struct

    public struct missionItem
    {
        /// <summary> PARAM1, see MAV_CMD enum </summary>
        public Single param1;
        /// <summary> PARAM2, see MAV_CMD enum </summary>
        public Single param2;
        /// <summary> PARAM3, see MAV_CMD enum </summary>
        public Single param3;
        /// <summary> PARAM4, see MAV_CMD enum </summary>
        public Single param4;
        /// <summary> PARAM5 / local: x position, global: latitude </summary>
        public Single x;
        ///// <summary> PARAM6 / y position: global: longitude </summary>
        public Single y;
        /// <summary> PARAM7 / z position: global: altitude (relative or absolute, depending on frame. </summary>
        public Single z;
        /// <summary> Sequence </summary>
        public UInt16 seq;
        /// <summary> The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs </summary>
        public UInt16 command;
        /// <summary> System ID </summary>
        public byte target_system;
        /// <summary> Component ID </summary>
        public byte target_component;
        /// <summary> The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h </summary>
        public byte frame;
        /// <summary> false:0, true:1 </summary>
        public byte current;
        /// <summary> autocontinue to next wp </summary>
        public byte autocontinue;

    };

    #endregion

}
