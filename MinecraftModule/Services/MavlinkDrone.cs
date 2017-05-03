using MavLink;
using MinecraftModule.Services;
using System;
using System.ComponentModel;
using System.IO.Ports;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using static MinecraftModule.Services.DroneModel;

namespace MultiDroneGCS1.Models
{
    public class MavlinkDrone : Drone
    {

        #region Fields and Const

        private byte seqNo;
        public byte txPacketSequence;

        public const byte SYSTEM_ID = 255;
        public const byte COMPONENT_ID = (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER;
        public AutoResetEvent ackLock = new AutoResetEvent(false);
        BackgroundWorker tcpReceiveBGW = new BackgroundWorker();

        public bool stopConnection = false;

        MavlinkParser mavlinkParser;

        #endregion Fields and Const

        #region Properties

        public override DroneStatus Status
        {
            get
            {
                return status;
            }
            set
            {
                SetProperty(ref this.status, value);
            }
        }

        #endregion Properties

        #region Methods

        #region Init Methods

        /// <summary> Initiate Data Pulling </summary>
        public override bool InitiatePreSettings()
        {
            CreateReqDataStream(MAV_DATA_STREAM.MAV_DATA_STREAM_EXTENDED_STATUS);
            CreateReqDataStream(MAV_DATA_STREAM.MAV_DATA_STREAM_POSITION);
            CreateReqDataStream(MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA1);
            CreateReqDataStream(MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA2);
            CreateReqDataStream(MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA3);
            CreateReqDataStream(MAV_DATA_STREAM.MAV_DATA_STREAM_RAW_SENSORS);
            CreateReqDataStream(MAV_DATA_STREAM.MAV_DATA_STREAM_RC_CHANNELS);

            ParamRequestList();

            return true;
        }

        /// <summary> Connect Vehicle </summary>
        public override void Connect()
        {
            if (StartConnection(this))
            {
                IsConnected = true;
                MyDebug.WriteLine("Connected");

                mavlinkParser = new MavlinkParser();

                if (serialPort != null && serialPort.IsOpen == true)
                {
                    serialPort.DataReceived += SerialPortDataReceived;
                }
                else if (tcpClient != null)
                {
                    tcpReceiveBGW.DoWork += StartTCPClientReceive;
                    if (!tcpReceiveBGW.IsBusy)
                    {
                        tcpReceiveBGW.RunWorkerAsync();
                    }
                    else
                    {
                        MyDebug.WriteLine("TCP BGW already running");
                    }
                }

                Thread.Sleep(100);

                InitiatePreSettings();
            }
        }

        /// <summary> Set Data to Request For </summary>
        public bool CreateReqDataStream(MAV_DATA_STREAM steamType)
        {
            //Init New Packet and Packet Message
            Msg_request_data_stream msg = new Msg_request_data_stream();

            msg.req_message_rate = 10;
            msg.req_stream_id = (byte)steamType;
            msg.start_stop = 0x01;

            SendDataStream(DoFrame(msg));

            return true;
        }

        public override bool ParamRequestList()
        {
            Msg_param_request_list msg = new Msg_param_request_list() { target_system = 0x01, target_component = 0x01 };
            SendDataStream(DoFrame(msg));
            return true;
        }

        public override bool ParamSet(byte[] id, float paramValue, MAV_PARAM_TYPE paramType)
        {
            Msg_param_set msg = new Msg_param_set()
            {
                target_system = 0x01,
                target_component = 0x01,
                
                param_value = paramValue,
                param_type = (byte)paramType
            };
            //msg.param_id[15] = (byte)'\0';
            Array.Copy(id, msg.param_id, id.Length);


            SendDataStream(DoFrame(msg));
            return true;
        }

        #endregion Init Methods

        #region Frame Contruction Methods

        /// <summary> Creates a Mavlink Packet </summary>
        private MavlinkPacket PreparePacket(MavlinkMessage msg)
        {
            MavlinkPacket packet = new MavlinkPacket() { SystemId = 255, ComponentId = (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER, Message = msg, SequenceNumber = seqNo++ };

            return packet;
        }

        private byte[] DoFrame(MavlinkPacket thisPacket)
        {

            var bytes = this.Serialize(thisPacket.Message);

            return ModifyToLinkLayer(bytes);
        }

        private byte[] DoFrame(MavlinkMessage thisMessage)
        {
            MavlinkPacket thisPacket = PreparePacket(thisMessage);

            var bytes = this.Serialize(thisPacket.Message);

            return ModifyToLinkLayer(bytes);
        }

        private byte[] Serialize(MavlinkMessage message)
        {
            var buff = new byte[256];

            buff[0] = (byte)SYSTEM_ID;
            buff[1] = (byte)COMPONENT_ID;

            var endPos = 3;
            var msgId = message.Serialize(buff, ref endPos);

            buff[2] = (byte)msgId;

            var resultBytes = new byte[endPos];
            Array.Copy(buff, resultBytes, endPos);

            return resultBytes;
        }

        private byte[] ModifyToLinkLayer(byte[] packetData)
        {
            var outBytes = new byte[packetData.Length + 5];

            outBytes[0] = MavlinkSettings.ProtocolMarker;
            outBytes[1] = (byte)(packetData.Length - 3);  // 3 bytes for sequence, id, msg type which this 
                                                          // layer does not concern itself with
            outBytes[2] = unchecked(txPacketSequence++);

            int i;

            for (i = 0; i < packetData.Length; i++)
            {
                outBytes[i + 3] = packetData[i];
            }

            // Check the CRC. Does not include the starting byte but does include the length
            var crc1 = Mavlink_Crc.Calculate(outBytes, 1, (UInt16)(packetData.Length + 2));

            if (MavlinkSettings.CrcExtra)
            {
                var possibleMsgId = outBytes[5];
                var extra = MavLinkSerializer.Lookup[possibleMsgId];
                crc1 = Mavlink_Crc.CrcAccumulate(extra.CrcExtra, crc1);
            }

            byte crc_high = (byte)(crc1 & 0xFF);
            byte crc_low = (byte)(crc1 >> 8);

            outBytes[i + 3] = crc_high;
            outBytes[i + 4] = crc_low;

            return outBytes;
        }

        #endregion Frame Construction Methods

        #region Parsing Methods

        public override void TcpClientDataReceived()
        {
            throw new NotImplementedException();
        }

        public override void SerialPortDataReceived(object sender, SerialDataReceivedEventArgs e)
        {

            SerialPort curSerialLink = (SerialPort)sender;

            if (!curSerialLink.IsOpen)
            {
                MyDebug.WriteLine($"Serial Receive Failed: Port {curSerialLink.PortName} is closed.");

                return;
            }

            int byteLen = curSerialLink.BytesToRead;
            byte[] buffer = new byte[byteLen];

            try
            {
                curSerialLink.Read(buffer, 0, byteLen);

                //Process Information

                mavlinkParser.ProcessPacketReceived(buffer, ref status, ref AckLock);
            }
            catch (Exception ex)
            {
                MyDebug.WriteLine("ReceiveData() Exception -->>" + ex.Message);
            }
        }

        public async void StartTCPClientReceive(object sender, DoWorkEventArgs eventArgs)
        {
            int counter = 0;
            int pingTimer = 0;

            try
            {
                await Task.Run(() =>
                {
                    while (!stopConnection)
                    {
                        if (tcpClient != null && tcpClient.Connected == true && tcpClient.Client != null)
                        {
                            if (tcpClient.Client.Poll(0, SelectMode.SelectRead))
                            {
                                NetworkStream stream = tcpClient.GetStream();

                                while (stream.DataAvailable == true)
                                {
                                    if (pingTimer++ >= 100)
                                    {
                                        TcpPing(Param1, 500);

                                        pingTimer = 0;
                                    }

                                    if (stopConnection)
                                    {
                                        break;
                                    }

                                    int bytesToRead = tcpClient.Available;

                                    IsConnected = true;

                                    byte[] dataBytes = new byte[bytesToRead];
                                    stream.Read(dataBytes, 0, dataBytes.Length);

                                    mavlinkParser.ProcessPacketReceived(dataBytes, ref status, ref AckLock);
                                }
                            }
                            else
                            {
                                if (AutoReconnect == true)
                                {
                                    counter++;

                                    // Attempt reconnect as incoming stream lost
                                    if (counter >= 10)
                                    {
                                        // If client instantiated, free up resources
                                        if (tcpClient.Client != null && tcpClient != null)
                                        {
                                            //tcpClient.Dispose();
                                            tcpClient.Close();
                                        }

                                        TcpConnect(Param1, Param2, Timeout);

                                        byte[] temp_data = new byte[] { 0xaa, 0x19, 0x00, 0x00 };

                                        SendDataStream(temp_data);

                                        if (this is MavlinkDrone)
                                        {
                                            this.InitiatePreSettings();
                                            Thread.Sleep(100);
                                        }

                                        counter = 0;
                                    }
                                }
                            }
                        }
                        else
                        {
                            // Attempt reconnect as client not connected
                            TcpConnect(Param1, Param2, Timeout);
                        }

                        Thread.Sleep(100);
                    }
                });
            }
            catch (Exception e)
            {
                MyDebug.WriteLine($"StartTCPClientReceive: {e.Message}");
            }
        }

        #endregion Parsing Methods

        #endregion Methods

        #region Commands

        public bool GenerateHeartBeat()
        {
            Msg_heartbeat heartBeat = new Msg_heartbeat();
            heartBeat.type = (byte)MAV_TYPE.MAV_TYPE_GCS;
            heartBeat.autopilot -= (byte)MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID;
            heartBeat.custom_mode = 0;
            heartBeat.system_status = 0;
            heartBeat.mavlink_version = 0;
            
            SendDataStream(DoFrame(heartBeat));

            return true;
        }

        public override bool Arm()
        {
            Msg_command_long msg = new Msg_command_long();
            msg.target_system = 0x01;
            msg.target_component = 0x01;
            msg.command = (UInt16)MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM;
            msg.confirmation = 0;
            msg.param1 = 1;
            msg.param2 = 0;
            msg.param3 = 0;
            msg.param4 = 0;
            msg.param5 = 0;
            msg.param6 = 0;
            msg.param7 = 0;

            SendDataStream(DoFrame(msg));

            return true;
        }

        public override bool Disarm()
        {
            Msg_command_long msg = new Msg_command_long();
            msg.target_system = 0x01;
            msg.target_component = 0x01;
            msg.command = (UInt16)MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM;
            msg.confirmation = 0;
            msg.param1 = 0;
            msg.param2 = 0;
            msg.param3 = 0;
            msg.param4 = 0;
            msg.param5 = 0;
            msg.param6 = 0;
            msg.param7 = 0;

            SendDataStream(DoFrame(msg));

            return true;
        }

        public override bool Land()
        {
            // Switch to Land flight mode
            Msg_command_long msg0 = new Msg_command_long();
            msg0.target_system = 0x01;
            msg0.target_component = 0x01;
            msg0.command = (UInt16)176;
            msg0.confirmation = 0;
            msg0.param1 = 209;
            msg0.param2 = 4;
            msg0.param3 = 6;
            msg0.param4 = 0;
            msg0.param5 = 0;
            msg0.param6 = 0;
            msg0.param7 = 0;

            SendDataStream(DoFrame(msg0));

            return true;
        }

        public override bool Launch()
        {
            Msg_command_long msg = new Msg_command_long();
            msg.target_system = 0x01;
            msg.target_component = 0x01;
            //msg.command = (UInt16)MAV_CMD.MAV_CMD_DO_REPOSITION;
            //msg.confirmation = 0;
            //msg.param1 = -1.0f;
            //msg.param2 = 1;
            //msg.param3 = 0;
            //msg.param4 = float.NaN; //NaN
            //msg.param5 = (float)status.latitudeRaw; // actual
            //msg.param6 = (float)status.longitudeRaw; // actual
            //msg.param7 = (float)status.altitude + 5;

            msg.command = (UInt16)MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT;
            msg.confirmation = 0;
            msg.param1 = 10.0f;
            msg.param2 = msg.param3 = msg.param4 = msg.param5 = msg.param6 = 0;
            msg.param7 = status.altitude + 5;

            SendDataStream(DoFrame(msg));

            return true;
        }

        public override bool ReturnToHome()
        {
            throw new NotImplementedException();
        }

        public override bool StartWaypointMission()
        {
            Msg_set_mode msg = new Msg_set_mode();

            msg.target_system = 0x01;
            msg.base_mode = (byte)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            msg.custom_mode = (byte)MAV_AUTOPILOT_MODE.AUTO;

            SendDataStream(DoFrame(msg));

            return true;
        }

        public override bool StopWaypointMission()
        {
            throw new NotImplementedException();
        }

        public override bool DownloadWaypointMission()
        {
            Msg_mission_request_list msg0 = new Msg_mission_request_list();
            msg0.target_system = 0x01;
            msg0.target_component = 0x01;

            SendDataStream(DoFrame(msg0));

            if (!AckLock.WaitOne(700))
            {
                MessageBox.Show("The ack timed out.");
                return false;
            }

            MyDebug.WriteLine($"Mission Count Received: {status.waypointCount}");

            for (int i = 0; i < status.waypointCount; i++)
            {
                Msg_mission_request msg = new Msg_mission_request();
                msg.target_component = 0x01;
                msg.target_system = 0x01;
                msg.seq = (ushort)i;
                SendDataStream(DoFrame(msg));

                AckLock.WaitOne();
            }

            Msg_mission_ack msg1 = new Msg_mission_ack();
            msg1.target_component = 0x01;
            msg1.target_system = 0x01;
            msg1.type = (byte)MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;
            SendDataStream(DoFrame(msg1));

            return true;
        }

        public override bool StartFollowMeMission()
        {
            throw new NotImplementedException();
        }

        public override bool StopFollowMeMission()
        {
            Msg_set_mode msg = new Msg_set_mode();

            msg.target_system = 0x01;
            msg.base_mode = (byte)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            msg.custom_mode = (byte)MAV_AUTOPILOT_MODE.STABILIZE;

            SendDataStream(DoFrame(msg));

            return true;
        }

        public override bool UploadWaypointMission(int count)
        {
            Msg_mission_count msg0 = new Msg_mission_count();
            msg0.target_system = 0x01;
            msg0.target_component = 0x01;
            msg0.count = (ushort)(count);

            SendDataStream(DoFrame(msg0));

            if (!AckLock.WaitOne(1000))
            {
                MessageBox.Show("The ack timed out.");
                //return false;
            }

            for (int index = 0; true; index++)
            {
                index = status.waypointCount;

                //if (index == 0)
                //{
                //    MessageBox.Show("Upload Failed, Please Select Waypoints Before Uploading!");
                //    return false;
                //}

                Msg_mission_item msg = new Msg_mission_item();
                msg.target_system = 0x01;
                msg.target_component = 0x01;
                msg.seq = (ushort)(index);
                msg.frame = (byte)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
                msg.command = 0x10;
                msg.current = 0;
                msg.autocontinue = 1;
                msg.param1 = (float)status.waypoints[index].Delay;
                msg.param2 = 0;
                msg.param3 = 0;
                msg.param4 = 0;
                msg.x = (float)status.waypoints[index].Lat;
                msg.y = (float)status.waypoints[index].Lng;
                msg.z = (float)status.waypoints[index].Alt;

                MyDebug.ConsoleWriteLine($"Sending WP index {index}");

                SendDataStream(DoFrame(msg));

                if (index == status.waypoints.Count - 1)
                {
                    return true;
                }

                //Thread.Sleep(100);
                if (!AckLock.WaitOne(600))
                {
                    MyDebug.ConsoleWriteLine($"Ack for waypoint {index} timeout");
                }
            }

            status.waypointCount = 0;
            status.waypoints.Clear();

            return true;
        }

        public override bool FlyToHere(double lat, double lng, float alt)
        {
            Msg_command_long msg = new Msg_command_long();
            msg.target_system = 0x01;
            msg.target_component = 0x01;
            msg.command = (UInt16)MAV_CMD.MAV_CMD_DO_REPOSITION;
            msg.confirmation = 0;
            msg.param1 = -1.0f;
            msg.param2 = 1;
            msg.param3 = 0;
            msg.param4 = float.NaN; //NaN
            msg.param5 = (float)lat; // actual
            msg.param6 = (float)lng; // actual
            msg.param7 = (float)alt;

            SendDataStream(DoFrame(msg));

            return true;
        }

        public override bool HighLaunch(float alt)
        {
            // Switch to Hold flight mode
            Msg_command_long msg0 = new Msg_command_long();
            msg0.target_system = 0x01;
            msg0.target_component = 0x01;
            msg0.command = (UInt16)176;
            msg0.confirmation = 0;
            msg0.param1 = 209;
            msg0.param2 = 4;
            msg0.param3 = 3;
            msg0.param4 = 0;
            msg0.param5 = 0;
            msg0.param6 = 0;
            msg0.param7 = 0;

            SendDataStream(DoFrame(msg0));

            Thread.Sleep(500);

            // MAV_CMD_NAV_TAKEOFF
            Msg_command_long msg = new Msg_command_long();
            msg.target_system = 0x01;
            msg.target_component = 0x01;
            msg.command = (UInt16)3;
            msg.confirmation = 0;
            msg.param1 = -1.0f;
            msg.param2 = 1;
            msg.param3 = 0;
            msg.param4 = float.NaN; //NaN
            msg.param5 = float.NaN; // actual
            msg.param6 = float.NaN; // actual
            msg.param7 = (float)alt;

            SendDataStream(DoFrame(msg));

            return true;
        }

        public override bool NudgeInDirection(Compass direction, double distance)
        {
            float offsetLat = 0;
            float offsetLng = 0;

            if (direction == Compass.NORTH)
            {
                offsetLat = (float)distance * 0.00001f;
            }
            else if (direction == Compass.SOUTH)
            {
                offsetLat = (float)distance * -0.00001f;
            }
            else if (direction == Compass.EAST)
            {
                offsetLng = (float)distance * 0.00001f;
            }
            else if (direction == Compass.WEST)
            {
                offsetLng = (float)distance * -0.00001f;
            }
            else
            {
                MyDebug.WriteLine("Invalid Compass Direction");
            }




            Msg_command_long msg = new Msg_command_long();
            msg.target_system = 0x01;
            msg.target_component = 0x01;
            msg.command = (UInt16)MAV_CMD.MAV_CMD_DO_REPOSITION;
            msg.confirmation = 0;
            msg.param1 = -1.0f;
            msg.param2 = 1;
            msg.param3 = 0;
            msg.param4 = float.NaN; //NaN
            msg.param5 = (float)status.latitude + offsetLat; // actual
            msg.param6 = (float)status.longitude + offsetLng; // actual
            msg.param7 = (float)float.NaN;

            SendDataStream(DoFrame(msg));

            return true;
        }

        public override bool StorageReadWrite(bool isWrite, bool reset)
        {
            Msg_command_long msg = new Msg_command_long();
            msg.target_system = 0x01;
            msg.target_component = 0x01;
            msg.command = (UInt16)MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE;
            msg.confirmation = 0;
            msg.param1 = (isWrite)? 1: 0;
            msg.param2 = (isWrite) ? 1 : 0;
            msg.param3 = 0;
            msg.param4 = 0;
            msg.param5 = 0;
            msg.param6 = 0;
            msg.param7 = 0;

            SendDataStream(DoFrame(msg));

            return true;

        }

        #endregion Commands

    }
}
