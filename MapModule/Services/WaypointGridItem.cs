using Prism.Mvvm;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MapModule.Services
{
    public class WaypointGridItem : BindableBase
    {
        private int index;
        private double lat;
        private double lng;
        private double alt = 5;
        private double delay = 0;

        public int Index
        {
            get { return index; }
            set
            {
                if (index != 0)
                {
                    SetProperty(ref index, value);
                }
                else
                {
                    index = value;
                }
            }
        }
        public double Lat
        {
            get { return lat; }
            set
            {
                if (lat != 0)
                {
                    SetProperty(ref lat, value);
                }
                else
                {
                    lat = value;
                }
            }
        }
        public double Lng
        {
            get
            {
                return lng;
            }
            set
            {
                if (lng != 0)
                {
                    SetProperty(ref lng, value);
                }
                else
                {
                    lng = value;
                }
            }
        }
        public double Alt
        {
            get
            {
                return alt;
            }
            set
            {
                if (alt != 0)
                {
                    SetProperty(ref alt, value);
                }
                else
                {
                    alt = value;
                }
            }
        }
        public double Delay
        {
            get
            {
                return delay;
            }
            set
            {
                if (value != 0)
                {
                    SetProperty(ref delay, value);
                }
                else
                {
                    delay = value;
                }
            }
        }

    }
}
