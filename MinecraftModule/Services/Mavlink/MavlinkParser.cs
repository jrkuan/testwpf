using MinecraftModule.Models;
using MinecraftModule.Services;
using System;
using System.Collections.ObjectModel;
using System.Threading;
using System.Windows;
using static MinecraftModule.Services.DroneModel;

namespace MavLink
{
    public class MavlinkParser
    {

        #region Fields and Const

        private byte[] leftovers;
        //Total Number of Packets Received Thus Far
        public UInt32 PacketsReceived { get; private set; }
        //Total Number of Packets Rejected Due to Failed CRC Checks
        public UInt32 BadCrcPacketsReceived { get; private set; }

        //Raised When Packet is Parsed Successfully
        public event PacketReceivedEventHandler MAVPacketReceived;
        //Raised When CRC Check Fails
        public event PacketCRCFailEventHandler PacketFailedCRC;
        //Raised When a Number of Bytes are Passed Over and Cannot be used to decode a packet
        public event PacketCRCFailEventHandler BytesUnused;

        public delegate void PacketCRCFailEventHandler(object sender, PacketCRCFailEventArgs e);
        public delegate void PacketReceivedEventHandler(object sender, MavlinkPacket e, byte id, ref DroneStatus status, ref AutoResetEvent AckLock);

        //

        #endregion Fields and Const

        #region Constructors

        public MavlinkParser()
        {
            MavLinkSerializer.SetDataIsLittleEndian(MavlinkSettings.IsLittleEndian);
            leftovers = new byte[] { };
            MAVPacketReceived += DecodePacketReceived;
        }

        #endregion Constructors

        #region Methods

        public void ProcessPacketReceived(byte[] packetBytes, ref DroneStatus status, ref AutoResetEvent AckLock)
        {

            uint i = 0;

            // copy the old and new into a contiguous array
            // This is pretty inefficient...
            var bytesToProcess = new byte[packetBytes.Length + leftovers.Length];
            int j = 0;

            for (i = 0; i < leftovers.Length; i++)
                bytesToProcess[j++] = leftovers[i];

            for (i = 0; i < packetBytes.Length; i++)
                bytesToProcess[j++] = packetBytes[i];

            i = 0;

            // we are going to loop and decode packets until we use up the data
            // at which point we will return. Hence one call to this method could
            // result in multiple packet decode events
            while (true)
            {
                // Hunt for the start char
                int huntStartPos = (int)i;

                while (i < bytesToProcess.Length && bytesToProcess[i] != MavlinkSettings.ProtocolMarker)
                    i++;

                if (i == bytesToProcess.Length)
                {
                    // No start byte found in all our bytes. Dump them, Exit.
                    leftovers = new byte[] { };
                    return;
                }

                if (i > huntStartPos)
                {
                    // if we get here then are some bytes which this code thinks are 
                    // not interesting and would be dumped. For diagnostics purposes,
                    // lets pop these bytes up in an event.
                    if (BytesUnused != null)
                    {
                        var badBytes = new byte[i - huntStartPos];
                        Array.Copy(bytesToProcess, huntStartPos, badBytes, 0, (int)(i - huntStartPos));
                        BytesUnused(this, new PacketCRCFailEventArgs(badBytes, bytesToProcess.Length - huntStartPos));
                    }
                }

                // We need at least the minimum length of a packet to process it. 
                // The minimum packet length is 8 bytes for acknowledgement packets without payload
                // if we don't have the minimum now, go round again
                if (bytesToProcess.Length - i < 8)
                {
                    leftovers = new byte[bytesToProcess.Length - i];
                    j = 0;
                    while (i < bytesToProcess.Length)
                        leftovers[j++] = bytesToProcess[i++];
                    return;
                }

                /*
                 * Byte order:
                 * 
                 * 0  Packet start sign	
                 * 1	 Payload length	 0 - 255
                 * 2	 Packet sequence	 0 - 255
                 * 3	 System ID	 1 - 255
                 * 4	 Component ID	 0 - 255
                 * 5	 Message ID	 0 - 255
                 * 6 to (n+6)	 Data	 (0 - 255) bytes
                 * (n+7) to (n+8)	 Checksum (high byte, low byte) for v0.9, lowbyte, highbyte for 1.0
                 *
                 */
                UInt16 payLoadLength = bytesToProcess[i + 1];

                // Now we know the packet length, 
                // If we don't have enough bytes in this packet to satisfy that packet lenghth,
                // then dump the whole lot in the leftovers and do nothing else - go round again
                if (payLoadLength > (bytesToProcess.Length - i - 8)) // payload + 'overhead' bytes (crc, system etc)
                {
                    // back up to the start char for next cycle
                    j = 0;

                    leftovers = new byte[bytesToProcess.Length - i];

                    for (; i < bytesToProcess.Length; i++)
                    {
                        leftovers[j++] = bytesToProcess[i];
                    }
                    return;
                }

                i++;

                // Check the CRC. Does not include the starting 'U' byte but does include the length
                var crc1 = Mavlink_Crc.Calculate(bytesToProcess, (UInt16)(i), (UInt16)(payLoadLength + 5));

                if (MavlinkSettings.CrcExtra)
                {
                    var possibleMsgId = bytesToProcess[i + 4];

                    if (!MavLinkSerializer.Lookup.ContainsKey(possibleMsgId))
                    {
                        // we have received an unknown message. In this case we don't know the special
                        // CRC extra, so we have no choice but to fail.

                        // The way we do this is to just let the procedure continue
                        // There will be a natural failure of the main packet CRC
                    }
                    else
                    {
                        var extra = MavLinkSerializer.Lookup[possibleMsgId];
                        crc1 = Mavlink_Crc.CrcAccumulate(extra.CrcExtra, crc1);
                    }
                }

                byte crcHigh = (byte)(crc1 & 0xFF);
                byte crcLow = (byte)(crc1 >> 8);

                byte messageCrcHigh = bytesToProcess[i + 5 + payLoadLength];
                byte messageCrcLow = bytesToProcess[i + 6 + payLoadLength];

                if (messageCrcHigh == crcHigh && messageCrcLow == crcLow)
                {
                    // This is used for data drop outs metrics, not packet windows
                    // so we should consider this here. 
                    // We pass up to subscribers only as an advisory thing
                    var rxPacketSequence = bytesToProcess[++i];
                    i++;
                    var packet = new byte[payLoadLength + 3];  // +3 because we are going to send up the sys and comp id and msg type with the data

                    for (j = 0; j < packet.Length; j++)
                        packet[j] = bytesToProcess[i + j];

                    var debugArray = new byte[payLoadLength + 7];
                    Array.Copy(bytesToProcess, (int)(i - 3), debugArray, 0, debugArray.Length);

                    //OnPacketDecoded(packet, rxPacketSequence, debugArray);

                    ProcessPacketBytes(packet, rxPacketSequence, ref status, ref AckLock);

                    PacketsReceived++;

                    // clear leftovers, just incase this is the last packet
                    leftovers = new byte[] { };

                    //  advance i here by j to avoid unecessary hunting
                    // todo: could advance by j + 2 I think?
                    i = i + (uint)(j + 2);
                }
                else
                {
                    var badBytes = new byte[i + 7 + payLoadLength];
                    Array.Copy(bytesToProcess, (int)(i - 1), badBytes, 0, payLoadLength + 7);

                    if (PacketFailedCRC != null)
                    {
                        PacketFailedCRC(this, new PacketCRCFailEventArgs(badBytes, (int)(bytesToProcess.Length - i - 1)));
                    }

                    BadCrcPacketsReceived++;
                }
            }
        }

        private void DecodePacketReceived(object sender, MavlinkPacket e, byte id, ref DroneStatus status, ref AutoResetEvent AckLock)
        {
            switch (id)
            {
                case (byte)MAVLINK_MSG_ID.HEARTBEAT:

                    var msg = (Msg_heartbeat)e.Message;
                    //_droneParams.type = msg.type;
                    //_droneParams.autoPilot = msg.autopilot;
                    //_droneParams.baseMode = msg.base_mode;
                    //_droneParams.customMode = (byte)msg.custom_mode;
                    //_droneParams.systemStatus = msg.system_status;
                    //_droneParams.mavlinkVersion = msg.mavlink_version;

                    //MyDebug.WriteLine(Convert.ToString(Status.type) + " "
                    //                + Convert.ToString(Status.autoPilot) + " "
                    //                + Convert.ToString(Status.baseMode) + " "
                    //                + Convert.ToString(Status.customMode) + " "
                    //                + Convert.ToString(Status.systemStatus) + " "
                    //                + Convert.ToString(Status.mavlinkVersion));

                    //MyDebug.WriteLine("Heartbeat");


                    //LT: TODO - Where to put this Alive?
                    //_droneParams.Alive = 1; //

                    break;

                case (byte)MAVLINK_MSG_ID.SYS_STATUS:

                    var msg1 = (Msg_sys_status)e.Message;
                    //_droneParams.onboardControlSensorsPresent = msg1.onboard_control_sensors_present;
                    //_droneParams.onboardControlSensorsEnabled = msg1.onboard_control_sensors_enabled;
                    //_droneParams.onboardControlSensorsHealth = msg1.onboard_control_sensors_health;
                    //_droneParams.load = msg1.load;

                    status.batteryVoltage = msg1.voltage_battery / 1000.0f;

                    break;

                case (byte)MAVLINK_MSG_ID.SYSTEM_TIME://2
                    var msg2 = (Msg_system_time)e.Message;
                    //_droneParams.timeUnixUsec = msg2.time_unix_usec;
                    status.uptime = msg2.time_boot_ms * 1000.0f;

                    //MyDebug.WriteLine(Convert.ToString(Status.timeUnixUsec) + " "
                    //        + Convert.ToString(Status.timeBootMs));
                    break;


                case (byte)MAVLINK_MSG_ID.PARAM_VALUE: //22

                    var msg3 = (Msg_param_value)e.Message;
                    //string type = "";

                    MAV_PARAM_TYPE paramType = (MAV_PARAM_TYPE)msg3.param_type;

                    //switch ((int)msg3.param_type)
                    //{
                    //    //case 0x01:
                    //    //type = "UINT8";
                    //    //    break;
                    //    //case 0x02:
                    //    //type = "INT8";
                    //    //    break;
                    //    //case 0x03:
                    //    //type = "UINT16";
                    //    //    break;
                    //    //case 0x04:                    
                    //    //type = "INT16";
                    //    //    break;
                    //    case 0x05:
                    //        type = "UINT32";
                    //        break;
                    //    case 0x06:
                    //        type = "INT32";
                    //        break;
                    //    //case 0x07:
                    //    //type = "UINT64";
                    //    //    break;
                    //    //case 0x08:
                    //    //type = "INT64";
                    //    //    break;
                    //    case 0x09:
                    //        type = "FLOAT32";
                    //        break;
                    //    //case 0x10:
                    //    //type = "FLOAT64";
                    //    //    break;
                    //    default:
                    //        type = "INT32";
                    //        MyDebug.ConsoleWriteLine("Unsupported Type");
                    //        break;
                    //}

                    //Application.Current.Dispatcher.BeginInvoke((Action)(() =>
                    //{

                    //    main.ParamView.ParamGridList.Add(
                    //        new ParamClass
                    //        {
                    //            Id = new string(System.Text.Encoding.UTF8.GetString(msg3.param_id).ToCharArray()),
                    //            Index = (ushort)msg3.param_index,
                                
                    //            //Type = type,
                    //            Value = BitConverter.GetBytes(msg3.param_value),
                    //            Count = (ushort)msg3.param_count,
                    //            ParamType = paramType
                    //        });

                    //    byte[] asd = msg3.param_id;

                    //}));

                    break;

                case (byte)MAVLINK_MSG_ID.GPS_RAW_INT: //24

                    var msg4 = (Msg_gps_raw_int)e.Message;
                    //_droneParams.timeUsecGPSRaw = msg4.time_usec;
                    status.gpsHealth = msg4.fix_type;
                    //_droneParams.fixType = msg4.fix_type;
                    //fixType 0 = No GPS Signal, 1 = Invalid GPS Signal, 2 = only X Y coordinates, 3 = 3d signals X Y Z coordinates, 
                    status.latitudeRaw = (double)msg4.lat / 10000000.0d;
                    status.longitudeRaw = (double)msg4.lon / 10000000.0d;

                    if (status.latitudeRaw + status.longitudeRaw != 0 && status.latitude + status.longitude == 0)
                    {
                        status.latitude = status.latitudeRaw;
                        status.longitude = status.longitudeRaw;
                    }

                    status.satelliteCount = msg4.satellites_visible;

                    break;

                case (byte)MAVLINK_MSG_ID.RAW_IMU: //27                    

                    var msg5 = (Msg_raw_imu)e.Message;

                    break;

                case (byte)MAVLINK_MSG_ID.SCALED_PRESSURE: //29

                    var msg6 = (Msg_scaled_pressure)e.Message;

                    break;

                case (byte)MAVLINK_MSG_ID.ATTITUDE://30

                    var msg7 = (Msg_attitude)e.Message;
                    //_droneParams.timeBootMsAtt = msg7.time_boot_ms;

                    //_droneParams.radPRY = new double[] { msg7.pitch, msg7.roll, msg7.yaw };
                    status.pitch = msg7.pitch;
                    status.roll = msg7.roll;
                    status.yaw = msg7.yaw;

                    break;
                case (byte)MAVLINK_MSG_ID.GLOBAL_POSITION_INT: //33


                    var msg8 = (Msg_global_position_int)e.Message;
                    //_droneParams.timeBootMsGlobalPosInt = msg8.time_boot_ms;

                    status.latitude = msg8.lat / 10000000f;
                    status.longitude = msg8.lon / 10000000f;
                    status.altitude = (float)msg8.alt / 1000.0f;

                    break;

                case (byte)MAVLINK_MSG_ID.RC_CHANNELS_RAW://35

                    var msg9 = (Msg_rc_channels_raw)e.Message;

                    break;

                case (byte)MAVLINK_MSG_ID.SERVO_OUTPUT_RAW://36

                    var msg10 = (Msg_servo_output_raw)e.Message;

                    break;

                //Parsing Waypoints Downloaded
                case (byte)MAVLINK_MSG_ID.MISSION_ITEM://39

                    var msg11 = (Msg_mission_item)e.Message;

                    WaypointClass waypointData = new WaypointClass()
                    {
                        Lat = msg11.x, Lng = msg11.y, Alt = msg11.z, Delay = msg11.param1
                    };

                    status.waypoints.Add(waypointData);
                    MyDebug.WriteLine($"received {msg11.command}");
                    AckLock.Set();

                    break;

                case (byte)MAVLINK_MSG_ID.MISSION_CURRENT://42

                    var msg12 = (Msg_mission_current)e.Message;
                    //_droneParams.MISSION_CURRENT_seq = msg12.seq;

                    //MyDebug.WriteLine(Convert.ToString(Status.seq));

                    break;

                case (byte)MAVLINK_MSG_ID.MISSION_COUNT://44

                    var msg13 = (Msg_mission_count)e.Message;
                    status.waypointCount = msg13.count;
                    
                    if (msg13.count != 0)
                    {
                        status.waypoints = new ObservableCollection<WaypointClass>();
                    }

                    AckLock.Set();

                    break;

                case (byte)MAVLINK_MSG_ID.MISSION_ACK://47

                    var msg14 = (Msg_mission_ack)e.Message;
                    //_droneParams.targetSystem = msg14.target_system;
                    //_droneParams.targetComponent = msg14.target_component;
                    //_droneParams.typeMissionAck = msg14.type;
                    //_droneParams.newMissionAck = true;

                    MyDebug.ConsoleWriteLine($"Mission ACK: {msg14.type}");
                    //Relate to Ack
                    //ProcessMavAck();

                    break;

                case (byte)MAVLINK_MSG_ID.NAV_CONTROLLER_OUTPUT://62

                    var msg15 = (Msg_nav_controller_output)e.Message;
                    break;

                case (byte)MAVLINK_MSG_ID.VFR_HUD://74

                    var msg16 = (Msg_vfr_hud)e.Message;
                    break;

                case (byte)MAVLINK_MSG_ID.COMMAND_ACK://77

                    var msg17 = (Msg_command_ack)e.Message;
                    break;

                case (byte)MAVLINK_MSG_ID.SCALED_IMU2://116

                    var msg18 = (Msg_scaled_imu2)e.Message;
                    break;

                case (byte)MAVLINK_MSG_ID.POWER_STATUS: //125

                    var msg19 = (Msg_power_status)e.Message;
                    break;

                case (byte)MAVLINK_MSG_ID.STATUSTEXT://253

                    byte[] text = new byte[50];
                    var msg28 = (Msg_statustext)e.Message;
                    byte severity = msg28.severity;
                    Array.Clear(text, 0, 50);
                    Buffer.BlockCopy(msg28.text, 0, text, 0, msg28.text.Length);

                    MyDebug.WriteLine(Convert.ToString(severity) + " " + System.Text.Encoding.ASCII.GetString(text, 0, Array.IndexOf<byte>(text, 0, 0, 50)));

                    if (severity <= 0x02)
                    {
                        MessageBox.Show(Convert.ToString(severity) + " " + System.Text.Encoding.ASCII.GetString(text, 0, Array.IndexOf<byte>(text, 0, 0, 50)));
                    }
                    else
                    {
                        MyDebug.ConsoleWriteLine(Convert.ToString(severity) + " " + System.Text.Encoding.ASCII.GetString(text, 0, Array.IndexOf<byte>(text, 0, 0, 50)));
                    }

                    break;

                case (byte)MAVLINK_MSG_ID.MISSION_REQUEST:

                    var msg29 = (Msg_mission_request)e.Message;
                    status.waypointCount = msg29.seq;

                    MyDebug.ConsoleWriteLine($"Drone requesting WP { msg29.seq.ToString()}");
                    AckLock.Set();
                    break;
            }
                    
        }

        private void ProcessPacketBytes(byte[] packetBytes, byte rxPacketSequence, ref DroneStatus status, ref AutoResetEvent AckLock)
        {
            //	 System ID	 1 - 255
            //	 Component ID	 0 - 255
            //	 Message ID	 0 - 255
            //   6 to (n+6)	 Data	 (0 - 255) bytes
            var packet = new MavlinkPacket
            {
                SystemId = packetBytes[0],
                ComponentId = packetBytes[1],
                SequenceNumber = rxPacketSequence,
                Message = this.Deserialize(packetBytes, 2)
            };

            if (MAVPacketReceived != null)
            {
                MAVPacketReceived(this, packet, packetBytes[2], ref status, ref AckLock);
            }

            // else do what?
        }

        private MavlinkMessage Deserialize(byte[] bytes, int offset)
        {
            // first byte is the mavlink 
            var packetNum = (int)bytes[offset + 0];
            var packetGen = MavLinkSerializer.Lookup[packetNum].Deserializer;
            return packetGen.Invoke(bytes, offset + 1);
        }

        #endregion Methods

    }
}
