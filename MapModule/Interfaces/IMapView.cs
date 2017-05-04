using MapModule.Views;
using System.Collections.ObjectModel;

namespace MapModule.Interfaces
{
    public interface IMapView
    {
        void SetModel(IMapViewModel model);

        void UpdateMapMarkers(ObservableCollection<CustomMapMarker> markers);
    }
}
