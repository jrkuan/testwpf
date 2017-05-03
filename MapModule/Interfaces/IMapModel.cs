﻿using MapModule.Views;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MapModule.Interfaces
{
    public interface IMapModel
    {
        ObservableCollection<CustomMapMarker> GetMarkers();
    }
}
