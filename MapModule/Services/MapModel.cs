using MapModule.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MapModule.Views;
using System.Collections.ObjectModel;

namespace MapModule.Services
{
    class MapModel : IMapModel
    {
        public ObservableCollection<CustomMapMarker> GetMarkers()
        {
            return null;
        }
    }
}
