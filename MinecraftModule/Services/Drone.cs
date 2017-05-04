using MavLink;
using MinecraftModule.Models;
using Prism.Mvvm;
using System;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO.Ports;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Threading;

namespace MinecraftModule.Services
{
    public abstract class Drone : DroneModel
    {
        
        public abstract DroneStatus Status { get; set; }

        public abstract void Connect();

        public abstract bool InitiatePreSettings();

        public abstract bool Arm();

        public abstract bool Disarm();

        public abstract bool Launch();

        public abstract bool Land();

        public abstract bool ReturnToHome();

        public abstract bool UploadWaypointMission(int count);

        public abstract bool StartWaypointMission();

        public abstract bool DownloadWaypointMission();

        public abstract bool NudgeInDirection(Compass direction, double distance);

        public abstract bool StopWaypointMission();

        public abstract bool StartFollowMeMission();

        public abstract bool StopFollowMeMission();

        public abstract bool HighLaunch(float alt);

        public abstract bool FlyToHere(double lat, double lng, float alt);

        public abstract void SerialPortDataReceived(object sender, SerialDataReceivedEventArgs e);

        public abstract void TcpClientDataReceived();

        public abstract bool ParamSet(byte[] id, float paramValue, MAV_PARAM_TYPE paramType);

        public abstract bool ParamRequestList();

        public abstract bool StorageReadWrite(bool isWrite, bool reset);
    }

    public class DroneModel : BindableBase
    {
        public struct DroneStatus
        {
            public float uptime;
            public double latitude;
            public double longitude;
            public double latitudeRaw;      // Estimated GPS data
            public double longitudeRaw;     // Estimated GPS data
            public float altitude;
            public float height;
            public byte gpsHealth;
            public int satelliteCount;
            public byte flightStatus;
            public double pitch;
            public double roll;
            public double yaw;
            public float velocity;
            public int batteryPercent;
            public float batteryVoltage;
            public int waypointCount;
            public byte flightMode;
            public ObservableCollection<WaypointClass> waypoints;

            //public DJIMission mission;
        }

        /// <summary>Defines the type of connection with the Drone.</summary>
        public enum ConnectionType
        {
            SERIAL,
            TCP
        }

        /// <summary>Defines the type to protocol used to communicate with the Drone.</summary>
        public enum DroneProtocol
        {
            MAVLINK,
            DJI,
            HOPELINK
        }

        public enum Compass
        {
            NORTH,
            SOUTH,
            EAST,
            WEST
        }

        public AutoResetEvent AckLock = new AutoResetEvent(false);

        public DroneModel()
        {
            status = new DroneStatus() { waypoints = new ObservableCollection<WaypointClass>() };
            AckLock = new AutoResetEvent(false);
        }

        public string Param1 { get; set; }
        public int Param2 { get; set; }
        public int Timeout { get; set; }
        public long RoundTripTime { get; set; }
        public bool StopTcpClient { get; set; }

        public ConnectionType ConnType { get; set; }

        /// <summary>Drone connection state.</summary>
        public bool IsConnected { get; set; }

        /// <summary>Set whether drone should reconnect automatically on connection loss.</summary>
        public bool AutoReconnect { get; set; }

        public SerialPort serialPort;

        public TcpClient tcpClient;

        protected DroneStatus status;

        public bool Disconnect()
        {
            if (tcpClient != null)
            {
                //tcpClient.Dispose();
                tcpClient = null;
            }
            tcpClient = null;
            if (serialPort != null)
            {
                serialPort.Close();
                serialPort = null;
            }
            IsConnected = false;
            return true;
        }

        /// <summary>Connect to either Serial or TCP. Parameters are hostname and port for TCP, and COM port and baudrate for Serial.</summary>
        protected bool StartConnection(ConnectionType connectionType, string param1, int param2, int timeout)
        {
            switch (connectionType)
            {
                case (ConnectionType.SERIAL):
                    tcpClient = null;
                    return SerialConnect(param1, param2, out serialPort);

                case (ConnectionType.TCP):
                    serialPort = null;
                    return TcpConnect(param1, param2, timeout);

                default:
                    tcpClient = null;
                    serialPort = null;
                    return false;
            }
        }

        /// <summary>Connect to either Serial or TCP. Parameters are hostname and port for TCP, and COM port and baudrate for Serial.</summary>
        protected bool StartConnection(Drone drone)
        {
            switch (drone.ConnType)
            {
                case (ConnectionType.SERIAL):
                    tcpClient = null;
                    return SerialConnect(drone.Param1, drone.Param2, out serialPort);

                case (ConnectionType.TCP):
                    serialPort = null;
                    return TcpConnect(drone.Param1, drone.Param2, drone.Timeout);

                default:
                    tcpClient = null;
                    serialPort = null;
                    return false;
            }
        }

        public bool SendDataStream(byte[] data)
        {
            try
            {
                // Serial communication
                if (serialPort != null && tcpClient == null)
                {
                    serialPort.Write(data, 0, data.Length);
                    return true;
                }
                // TCP communication
                else if (serialPort == null && tcpClient != null)
                {
                    if (!tcpClient.Connected)
                    {
                        return false;
                    }
                    else
                    {
                        NetworkStream dataStream = tcpClient.GetStream();
                        dataStream.Write(data, 0, data.Length);
                        return true;
                    }
                }
                else
                {
                    return false;
                }
            }
            catch
            {
                return false;
            }
        }

        #region Serial Communication

        public bool SerialConnect(string comPort, int baudrate, out SerialPort serialPort)
        {
            serialPort = null;

            string[] ports = SerialPort.GetPortNames();

            if (ports.Length != 0)
            {
                for (int i = 0; i < ports.Length; i++)
                {
                    if (ports[i].Equals(comPort))
                    {
                        try
                        {
                            serialPort = new SerialPort(comPort, baudrate, Parity.None, 8, StopBits.One);
                            IsConnected = true;
                        }
                        catch (Exception e)
                        {
                            MyDebug.WriteLine(e.Message);
                            IsConnected = false;
                        }
                        

                        break;
                    }
                }

                //check if _serialPort not constructed means no connection
                if (serialPort != null)
                {
                    //serialPort.DataReceived += SerialPortDataReceived;
                    try
                    {
                        serialPort.Open();
                        return true;
                    }
                    catch (Exception e)
                    {
                    
                        MyDebug.WriteLine(e.Message);
                    }
                    
                }
                else
                {
                    MyDebug.WriteLine("Serial Port not found.");
                }
            }
            else
            {
                MyDebug.WriteLine("No Serial Port Avaliable");
            }

            return false;
        }

        #endregion Serial Communication

        #region TCP Connection

        public bool TcpConnect(string hostName, int port, int timeout)
        {
            IPAddress ip;

            if (IPAddress.TryParse(hostName, out ip))
            {

                if (TcpPing(hostName, timeout))
                {
                    try
                    {
                        tcpClient = new TcpClient(hostName, port);
                        IsConnected = true;
                        return true;
                    }
                    catch (Exception exception)
                    {
                        MyDebug.WriteLine(exception.Message);
                    }
                    
                }
                else
                {
                    tcpClient = null;
                    return false;
                }
            }
            else
            {
                MyDebug.WriteLine("Invalid IP Address");
            }
            return false;
        }

        public bool TcpPing(string host, int timeout)
        {
            if (timeout < 10)
            {
                timeout = 100;
            }

            try
            {
                PingReply pingReply = new Ping().Send(host, timeout);
                return pingReply.Status == IPStatus.Success;
            }
            catch
            {
                Debug.WriteLine("TCP Connection Failed -----------------> Ping Unsuccessful!");
            }
            return false;
        }

        #endregion TCP Connection

    }
}
