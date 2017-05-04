using GMap.NET;
using MapModule.Views;
using System.Collections.ObjectModel;

namespace MapModule.Interfaces
{
    public interface IMapViewModel
    {
        ObservableCollection<CustomMapMarker> MapMarkers { get; }

        void UpdateCurrentMouseLatLng(PointLatLng currentMouseLatLng);

        IMapView View { get; }
                    
    }
}
