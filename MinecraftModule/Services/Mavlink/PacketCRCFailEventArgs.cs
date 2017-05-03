using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MavLink
{
    public class PacketCRCFailEventArgs : EventArgs
    {
        #region Fields and Const

        public byte[] BadPacket;
        public int Offset;

        #endregion

        #region Constructors

        public PacketCRCFailEventArgs(byte[] badPacket, int offset)
        {
            BadPacket = badPacket;
            Offset = offset;
        }

        #endregion

    }
}
