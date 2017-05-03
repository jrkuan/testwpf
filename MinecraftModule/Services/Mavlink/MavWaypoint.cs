using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MavLink
{
    public class MavWaypoint
    {

        #region Fields and Const

        public double lat;
        public double lng;
        public double alt;
        public double dwellTime;

        #endregion

        #region Constructors

        public MavWaypoint(double lat, double lng, double alt, double dwellTime)
        {
            this.lat = lat;
            this.lng = lng;
            this.alt = alt;
            this.dwellTime = dwellTime;
        }

        #endregion

    }
}
