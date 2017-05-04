using MapModule.Views;
using System.Collections.ObjectModel;

namespace MapModule.Interfaces
{
    public interface IMapModel
    {
        ObservableCollection<CustomMapMarker> GetMarkers();
    }
}
