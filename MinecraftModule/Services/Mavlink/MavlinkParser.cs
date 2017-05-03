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
                    //_droneParams.voltageBattery = msg1.voltage_battery;
                    //_droneParams.currentBattery = msg1.current_battery;
                    //_droneParams.batteryRemaining = msg1.battery_remaining;
                    //_droneParams.dropRateComm = msg1.drop_rate_comm;
                    //_droneParams.errorsComm = msg1.errors_comm;
                    //_droneParams.errorsCount1 = msg1.errors_count1;
                    //_droneParams.errorsCount2 = msg1.errors_count2;
                    //_droneParams.errorsCount3 = msg1.errors_count3;
                    //_droneParams.errorsCount4 = msg1.errors_count4;

                    //MyDebug.WriteLine(Convert.ToString(Status.onboardControlSensorsPresent) + " "
                    //        + Convert.ToString(Status.onboardControlSensorsEnabled) + " "
                    //        + Convert.ToString(Status.onboardControlSensorsHealth) + " "
                    //        + Convert.ToString(Status.load) + " "
                    //        + Convert.ToString(Status.voltageBattery) + " "
                    //        + Convert.ToString(Status.currentBattery) + " "
                    //        + Convert.ToString(Status.batteryRemaining) + " "
                    //        + Convert.ToString(Status.dropRateComm) + " "
                    //        + Convert.ToString(Status.errorsComm) + " "
                    //        + Convert.ToString(Status.errorsCount1) + " "
                    //        + Convert.ToString(Status.errorsCount2) + " "
                    //        + Convert.ToString(Status.errorsCount3) + " "
                    //        + Convert.ToString(Status.errorsCount4));
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

                    ////_droneParams.Alt = (float)msg4.alt/1000f;
                    //_droneParams.eph = msg4.eph;
                    //_droneParams.epv = msg4.epv;
                    //_droneParams.vel = msg4.vel;
                    //_droneParams.cog = msg4.cog;
                    //_droneParams.satellitesVisible = msg4.satellites_visible;
                    status.satelliteCount = msg4.satellites_visible;

                    //MyDebug.WriteLine("GPS_RAW_INT => " + Convert.ToString(_droneParams.timeUsecGPSRaw) + " "
                    //                                  + Convert.ToString(_droneParams.fixType) + " "
                    //                                  + Convert.ToString(_droneParams.lat) + " "
                    //                                  + Convert.ToString(_droneParams.lon) + " "
                    //                                  + Convert.ToString(_droneParams.alt) + " "
                    //                                  + Convert.ToString(_droneParams.eph) + " "
                    //                                  + Convert.ToString(_droneParams.epv) + " "
                    //                                  + Convert.ToString(_droneParams.vel) + " "
                    //                                  + Convert.ToString(_droneParams.cog) + " "
                    //                                  + Convert.ToString(_droneParams.satellitesVisible));

                    break;

                case (byte)MAVLINK_MSG_ID.RAW_IMU: //27                    



                    var msg5 = (Msg_raw_imu)e.Message;
                    //_droneParams.timeUsecIMURaw = msg5.time_usec;
                    //_droneParams.xaccIMURaw = msg5.xacc;
                    //_droneParams.yaccIMURaw = msg5.yacc;
                    //_droneParams.zaccIMURaw = msg5.zacc;
                    ////_droneParams.radPRY = new double[] { msg5.ygyro, msg5.xgyro, msg5.zgyro };
                    //_droneParams.xmagIMURaw = msg5.xmag;
                    //_droneParams.ymagIMURaw = msg5.ymag;
                    //_droneParams.zmagIMURaw = msg5.zmag;

                    //MyDebug.WriteLine(Convert.ToString(Status.timeUsecIMURaw) + " "
                    //                + Convert.ToString(Status.xaccIMURaw) + " "
                    //                + Convert.ToString(Status.yaccIMURaw) + " "
                    //                + Convert.ToString(Status.zaccIMURaw) + " "
                    //                + Convert.ToString(Status.xgyroIMURaw) + " "
                    //                + Convert.ToString(Status.ygyroIMURaw) + " "
                    //                + Convert.ToString(Status.zgyroIMURaw) + " "
                    //                + Convert.ToString(Status.xmagIMURaw) + " "
                    //                + Convert.ToString(Status.ymagIMURaw) + " "
                    //                + Convert.ToString(Status.zmagIMURaw));
                    break;

                case (byte)MAVLINK_MSG_ID.SCALED_PRESSURE: //29

                    var msg6 = (Msg_scaled_pressure)e.Message;
                    //_droneParams.timeBootMsScaledPress = msg6.time_boot_ms;
                    //_droneParams.pressAbs = msg6.press_abs;
                    //_droneParams.pressDiff = msg6.press_diff;
                    //_droneParams.temperature = msg6.temperature;

                    //MyDebug.WriteLine("29 => " + Convert.ToString(_droneParams.timeBootMsScaledPress) + " "
                    //                + Convert.ToString(_droneParams.pressAbs) + " "
                    //                + Convert.ToString(_droneParams.pressDiff) + " "
                    //                + Convert.ToString(_droneParams.temperature));
                    break;

                case (byte)MAVLINK_MSG_ID.ATTITUDE://30

                    var msg7 = (Msg_attitude)e.Message;
                    //_droneParams.timeBootMsAtt = msg7.time_boot_ms;

                    //_droneParams.radPRY = new double[] { msg7.pitch, msg7.roll, msg7.yaw };
                    status.pitch = msg7.pitch;
                    status.roll = msg7.roll;
                    status.yaw = msg7.yaw;
                    //_droneParams.roll = msg7.roll;
                    //_droneParams.pitch = msg7.pitch;
                    //_droneParams.yaw = msg7.yaw;
                    //_droneParams.rollspeed = msg7.rollspeed;
                    //_droneParams.pitchspeed = msg7.pitchspeed;
                    //_droneParams.yawspeed = msg7.yawspeed;

                    //MyDebug.WriteLine(Convert.ToString(_droneParams.timeBootMsAtt) + " "
                    //                + Convert.ToString(_droneParams.radPRY[2]) + " "
                    //                + Convert.ToString(_droneParams.radPRY[1]) + " "
                    //                + Convert.ToString(_droneParams.radPRY[0]) + " "
                    //                + Convert.ToString(_droneParams.rollspeed) + " "
                    //                + Convert.ToString(_droneParams.pitchspeed) + " "
                    //                + Convert.ToString(_droneParams.yawspeed));
                    break;
                case (byte)MAVLINK_MSG_ID.GLOBAL_POSITION_INT: //33


                    var msg8 = (Msg_global_position_int)e.Message;
                    //_droneParams.timeBootMsGlobalPosInt = msg8.time_boot_ms;

                    status.latitude = msg8.lat / 10000000f;
                    status.longitude = msg8.lon / 10000000f;
                    status.altitude = (float)msg8.alt / 1000.0f;
                    //_droneParams.Rlv_Alt = msg8.relative_alt / 1000;
                    //_droneParams.vx = msg8.vx;
                    //_droneParams.vy = msg8.vy;
                    //_droneParams.vz = msg8.vz;
                    //_droneParams.hdg = msg8.hdg;

                    //MyDebug.WriteLine(Convert.ToString(Status.timeBootMsGlobalPosInt) + " "
                    //                + Convert.ToString(Status.latGlobalPosInt) + " "
                    //                + Convert.ToString(Status.lonGlobalPosInt) + " "
                    //                + Convert.ToString(Status.altGlobalPosInt) + " "
                    //                + Convert.ToString(Status.relativeAlt) + " "
                    //                + Convert.ToString(Status.vx) + " "
                    //                + Convert.ToString(Status.vy) + " "
                    //                + Convert.ToString(Status.vz) + " "
                    //                + Convert.ToString(Status.hdg));
                    break;

                case (byte)MAVLINK_MSG_ID.RC_CHANNELS_RAW://35

                    var msg9 = (Msg_rc_channels_raw)e.Message;
                    //_droneParams.timeBootMsRCChannelRaw = msg9.time_boot_ms;
                    //_droneParams.port = msg9.port;
                    //_droneParams.chan1Raw = msg9.chan1_raw;
                    //_droneParams.chan2Raw = msg9.chan2_raw;
                    //_droneParams.chan3Raw = msg9.chan3_raw;
                    //_droneParams.chan4Raw = msg9.chan4_raw;
                    //_droneParams.chan5Raw = msg9.chan5_raw;
                    //_droneParams.chan6Raw = msg9.chan6_raw;
                    //_droneParams.chan7Raw = msg9.chan7_raw;
                    //_droneParams.chan8Raw = msg9.chan8_raw;
                    //_droneParams.rssi = msg9.rssi;

                    //MyDebug.WriteLine(Convert.ToString(Status.timeBootMsRCChannelRaw) + " "
                    //                + Convert.ToString(Status.port) + " "
                    //                + Convert.ToString(Status.chan1Raw) + " "
                    //                + Convert.ToString(Status.chan2Raw) + " "
                    //                + Convert.ToString(Status.chan3Raw) + " "
                    //                + Convert.ToString(Status.chan4Raw) + " "
                    //                + Convert.ToString(Status.chan5Raw) + " "
                    //                + Convert.ToString(Status.chan6Raw) + " "
                    //                + Convert.ToString(Status.chan7Raw) + " "
                    //                + Convert.ToString(Status.chan8Raw) + " "
                    //                + Convert.ToString(Status.rssi));
                    break;

                case (byte)MAVLINK_MSG_ID.SERVO_OUTPUT_RAW://36

                    var msg10 = (Msg_servo_output_raw)e.Message;
                    //_droneParams.timeUsecServoOutRaw = msg10.time_usec;
                    //_droneParams.portServoOutRaw = msg10.port;
                    //_droneParams.servo1Raw = msg10.servo1_raw;
                    //_droneParams.servo2Raw = msg10.servo2_raw;
                    //_droneParams.servo3Raw = msg10.servo3_raw;
                    //_droneParams.servo4Raw = msg10.servo4_raw;
                    //_droneParams.servo5Raw = msg10.servo5_raw;
                    //_droneParams.servo6Raw = msg10.servo6_raw;
                    //_droneParams.servo7Raw = msg10.servo7_raw;
                    //_droneParams.servo8Raw = msg10.servo8_raw;

                    //MyDebug.WriteLine(Convert.ToString(Status.timeUsecServoOutRaw) + " "
                    //                + Convert.ToString(Status.portServoOutRaw) + " "
                    //                + Convert.ToString(Status.servo1Raw) + " "
                    //                + Convert.ToString(Status.servo2Raw) + " "
                    //                + Convert.ToString(Status.servo3Raw) + " "
                    //                + Convert.ToString(Status.servo4Raw) + " "
                    //                + Convert.ToString(Status.servo5Raw) + " "
                    //                + Convert.ToString(Status.servo6Raw) + " "
                    //                + Convert.ToString(Status.servo7Raw) + " "
                    //                + Convert.ToString(Status.servo8Raw));
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

                    //_droneParams.MISSION_ITEM_seq = msg11.seq;
                    //_droneParams.MISSION_ITEM_frame = msg11.frame;
                    //_droneParams.MISSION_ITEM_command = msg11.command;
                    //_droneParams.MISSION_ITEM_current = msg11.current;
                    //_droneParams.MISSION_ITEM_autocontinue = msg11.autocontinue;
                    //_droneParams.MISSION_ITEM_param1 = msg11.param1;
                    //_droneParams.MISSION_ITEM_param2 = msg11.param2;
                    //_droneParams.MISSION_ITEM_param3 = msg11.param3;
                    //_droneParams.MISSION_ITEM_param4 = msg11.param4;
                    //_droneParams.MISSION_ITEM_x = msg11.x;
                    //_droneParams.MISSION_ITEM_y = msg11.y;
                    //_droneParams.MISSION_ITEM_z = msg11.z;
                    //_droneParams.newMissionRequestAns = true;

                    //MyDebug.WriteLine(_droneParams.MISSION_ITEM_x.ToString() + " "
                    //                + _droneParams.MISSION_ITEM_y.ToString() + " "
                    //                + _droneParams.MISSION_ITEM_z.ToString() + " "
                    //                + _droneParams.MISSION_ITEM_frame.ToString());
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
                    //_droneParams.MISSION_COUNT_count = msg13.count;
                    //_droneParams.newMissionRequestListAns = true;

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
                    //_droneParams.navRoll = msg15.nav_roll;
                    //_droneParams.navPitch = msg15.nav_pitch;
                    //_droneParams.navBearing = msg15.nav_bearing;
                    //_droneParams.targetBearing = msg15.target_bearing;
                    //_droneParams.wpDist = msg15.wp_dist;
                    //_droneParams.altError = msg15.alt_error;
                    //_droneParams.aspdError = msg15.aspd_error;
                    //_droneParams.xtrackError = msg15.xtrack_error;

                    //MyDebug.WriteLine(Convert.ToString(Status.navRoll) + " "
                    //                + Convert.ToString(Status.navPitch) + " "
                    //                + Convert.ToString(Status.navBearing) + " "
                    //                + Convert.ToString(Status.targetBearing) + " "
                    //                + Convert.ToString(Status.wpDist) + " "
                    //                + Convert.ToString(Status.altError) + " "
                    //                + Convert.ToString(Status.aspdError) + " "
                    //                + Convert.ToString(Status.xtrackError));
                    break;

                case (byte)MAVLINK_MSG_ID.VFR_HUD://74

                    var msg16 = (Msg_vfr_hud)e.Message;
                    //_droneParams.airspeed = msg16.airspeed;
                    //_droneParams.groundspeed = msg16.groundspeed;
                    //_droneParams.heading = msg16.heading;
                    //_droneParams.throttle = msg16.throttle;
                    //_droneParams.altVFRHUD = msg16.alt;
                    //_droneParams.climb = msg16.climb;

                    //MyDebug.WriteLine(Convert.ToString(Status.airspeed) + " "
                    //                + Convert.ToString(Status.groundspeed) + " "
                    //                + Convert.ToString(Status.heading) + " "
                    //                + Convert.ToString(Status.throttle) + " "
                    //                + Convert.ToString(Status.altVFRHUD) + " "
                    //                + Convert.ToString(Status.climb));
                    break;

                case (byte)MAVLINK_MSG_ID.COMMAND_ACK://77

                    var msg17 = (Msg_command_ack)e.Message;
                    //_droneParams.command = msg17.command;
                    //_droneParams.result = msg17.result;
                    //_droneParams.newCommandAck = true;

                    //MyDebug.WriteLine("COMMAND_ACK => " + Convert.ToString(_droneParams.command) + " " + Convert.ToString(_droneParams.result));

                    break;

                case (byte)MAVLINK_MSG_ID.SCALED_IMU2://116

                    var msg18 = (Msg_scaled_imu2)e.Message;
                    //_droneParams.timeBootMsScaledIMU2 = msg18.time_boot_ms;
                    //_droneParams.xaccScaledIMU2 = msg18.xacc;
                    //_droneParams.yaccScaledIMU2 = msg18.yacc;
                    //_droneParams.zaccScaledIMU2 = msg18.zacc;
                    //_droneParams.xgyroScaledIMU2 = msg18.xgyro;
                    //_droneParams.ygyroScaledIMU2 = msg18.ygyro;
                    //_droneParams.zgyroScaledIMU2 = msg18.zgyro;
                    //_droneParams.xmagScaledIMU2 = msg18.xmag;
                    //_droneParams.ymagScaledIMU2 = msg18.ymag;
                    //_droneParams.zmagScaledIMU2 = msg18.zmag;

                    //MyDebug.WriteLine("116 => " + Convert.ToString(_droneParams.timeBootMsScaledIMU2) + " "
                    //                + Convert.ToString(_droneParams.xaccScaledIMU2) + " "
                    //                + Convert.ToString(_droneParams.yaccScaledIMU2) + " "
                    //                + Convert.ToString(_droneParams.zaccScaledIMU2) + " "
                    //                + Convert.ToString(_droneParams.xgyroScaledIMU2) + " "
                    //                + Convert.ToString(_droneParams.ygyroScaledIMU2) + " "
                    //                + Convert.ToString(_droneParams.zgyroScaledIMU2) + " "
                    //                + Convert.ToString(_droneParams.xmagScaledIMU2) + " "
                    //                + Convert.ToString(_droneParams.ymagScaledIMU2) + " "
                    //                + Convert.ToString(_droneParams.zmagScaledIMU2));
                    break;

                case (byte)MAVLINK_MSG_ID.POWER_STATUS: //125

                    var msg19 = (Msg_power_status)e.Message;
                    //_droneParams.Vcc = msg19.Vcc;
                    //_droneParams.Vservo = msg19.Vservo;
                    //_droneParams.flags = msg19.flags;

                    //MyDebug.WriteLine(Convert.ToString(Status.Vcc) + " "
                    //                + Convert.ToString(Status.Vservo) + " "
                    //                + Convert.ToString(Status.flags));
                    break;

                //case (byte)MAVLINK_MSG_ID.MEMINFO://152

                //    var msg20 = (Msg_meminfo)e.Message;
                //    _droneParams.brkval = msg20.brkval;
                //    _droneParams.freemem = msg20.freemem;

                //    //MyDebug.WriteLine(Convert.ToString(Status.brkval) + " "
                //    //                + Convert.ToString(Status.freemem));
                //    break;

                //case (byte)MAVLINK_MSG_ID.MOUNT_STATUS://158

                //    var msg21 = (Msg_mount_status)e.Message;
                //    _droneParams.targetSystemMountStat = msg21.target_system;
                //    _droneParams.targetComponentMountStat = msg21.target_component;
                //    _droneParams.pointingA = msg21.pointing_a;
                //    _droneParams.pointingB = msg21.pointing_b;
                //    _droneParams.pointingC = msg21.pointing_c;

                //    //MyDebug.WriteLine(Convert.ToString(Status.targetSystem) + " "
                //    //                + Convert.ToString(Status.targetComponent) + " "
                //    //                + Convert.ToString(Status.pointingA) + " "
                //    //                + Convert.ToString(Status.pointingB) + " "
                //    //                + Convert.ToString(Status.pointingC));
                //    break;

                //case (byte)MAVLINK_MSG_ID.AHRS://163

                //    var msg22 = (Msg_ahrs)e.Message;
                //    _droneParams.omegaIx = msg22.omegaIx;
                //    _droneParams.omegaIy = msg22.omegaIy;
                //    _droneParams.omegaIz = msg22.omegaIz;
                //    _droneParams.accelWeight = msg22.accel_weight;
                //    _droneParams.renormVal = msg22.renorm_val;
                //    _droneParams.errorRp = msg22.error_rp;
                //    _droneParams.errorYaw = msg22.error_yaw;

                //    //MyDebug.WriteLine(Convert.ToString(Status.omegaIx) + " "
                //    //                + Convert.ToString(Status.omegaIy) + " "
                //    //                + Convert.ToString(Status.omegaIz) + " "
                //    //                + Convert.ToString(Status.accelWeight) + " "
                //    //                + Convert.ToString(Status.renormVal) + " "
                //    //                + Convert.ToString(Status.errorRp) + " "
                //    //                + Convert.ToString(Status.errorYaw));
                //    break;

                //case (byte)MAVLINK_MSG_ID.HWSTATUS://165

                //    var msg23 = (Msg_hwstatus)e.Message;
                //    _droneParams.VccHwStat = msg23.Vcc;
                //    _droneParams.I2Cerr = msg23.I2Cerr;

                //    //MyDebug.WriteLine(Convert.ToString(Status.VccHwStat) + " "
                //    //                + Convert.ToString(Status.I2Cerr));
                //    break;

                //case (byte)MAVLINK_MSG_ID.RANGEFINDER://173

                //    var msg24 = (Msg_rangefinder)e.Message;
                //    _droneParams.rangeFinderDistance = msg24.distance;
                //    _droneParams.rangeFinderVoltage = msg24.voltage;

                //    //MyDebug.WriteLine(Convert.ToString(Status.rangeFinderDistance) + " "
                //    //                + Convert.ToString(Status.rangeFinderVoltage));
                //    break;

                //case (byte)MAVLINK_MSG_ID.AHRS2://178

                //    var msg25 = (Msg_ahrs2)e.Message;
                //    _droneParams.rollAHRS2 = msg25.roll;
                //    _droneParams.pitchAHRS2 = msg25.pitch;
                //    _droneParams.yawAHRS2 = msg25.yaw;
                //    _droneParams.altitude = msg25.altitude;
                //    _droneParams.latAHRS2 = msg25.lat;
                //    _droneParams.lng = msg25.lng;

                //    //MyDebug.WriteLine(Convert.ToString(Status.rollAHRS2) + " "
                //    //                + Convert.ToString(Status.pitchAHRS2) + " "
                //    //                + Convert.ToString(Status.yawAHRS2) + " "
                //    //                + Convert.ToString(Status.altitude) + " "
                //    //                + Convert.ToString(Status.latAHRS2) + " "
                //    //                + Convert.ToString(Status.lng));
                //    break;

                //case (byte)MAVLINK_MSG_ID.EKF_STATUS_REPORT://193

                //    var msg26 = (Msg_ekf_status_report)e.Message;
                //    _droneParams.flagsEKF = msg26.flags;
                //    _droneParams.velocityVariance = msg26.velocity_variance;
                //    _droneParams.posHorizVariance = msg26.pos_horiz_variance;
                //    _droneParams.posVertVariance = msg26.pos_vert_variance;
                //    _droneParams.compassVariance = msg26.compass_variance;
                //    _droneParams.terrainAltVariance = msg26.terrain_alt_variance;

                //    //MyDebug.WriteLine(Convert.ToString(Status.flagsEKF) + " "
                //    //                + Convert.ToString(Status.velocityVariance) + " "
                //    //                + Convert.ToString(Status.posHorizVariance) + " "
                //    //                + Convert.ToString(Status.posVertVariance) + " "
                //    //                + Convert.ToString(Status.compassVariance) + " "
                //    //                + Convert.ToString(Status.terrainAltVariance));
                //    break;

                //case (byte)MAVLINK_MSG_ID.VIBRATION://241

                //    var msg27 = (Msg_vibration)e.Message;
                //    _droneParams.timeUsecVibration = msg27.time_usec;
                //    _droneParams.vibrationX = msg27.vibration_x;
                //    _droneParams.vibrationY = msg27.vibration_y;
                //    _droneParams.vibrationZ = msg27.vibration_z;
                //    _droneParams.clipping0 = msg27.clipping_0;
                //    _droneParams.clipping1 = msg27.clipping_1;
                //    _droneParams.clipping2 = msg27.clipping_2;

                //    //MyDebug.WriteLine(Convert.ToString(_droneParams.timeUsecVibration) + " "
                //    //                + Convert.ToString(_droneParams.vibrationX) + " "
                //    //                + Convert.ToString(_droneParams.vibrationY) + " "
                //    //                + Convert.ToString(_droneParams.vibrationZ) + " "
                //    //                + Convert.ToString(_droneParams.clipping0) + " "
                //    //                + Convert.ToString(_droneParams.clipping1) + " "
                //    //                + Convert.ToString(_droneParams.clipping2));
                //    break;

                case (byte)MAVLINK_MSG_ID.STATUSTEXT://253

                    byte[] text = new byte[50];
                    var msg28 = (Msg_statustext)e.Message;
                    byte severity = msg28.severity;
                    Array.Clear(text, 0, 50);
                    Buffer.BlockCopy(msg28.text, 0, text, 0, msg28.text.Length);

                    //bool bugCheck = String.Compare(System.Text.Encoding.ASCII.GetString(text, 0, Array.IndexOf<byte>(text, 0, 0, 50)), "ERR: wp index out of bounds", true) == 1;

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
