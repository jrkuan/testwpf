using MapModule.Interfaces;
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
