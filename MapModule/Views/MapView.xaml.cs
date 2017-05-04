using GMap.NET;
using MapModule.Interfaces;
using MapModule.Views;
using Prism.Commands;
using System.Diagnostics;
using System.Windows.Controls;
using System.Windows.Input;
using System;
using System.Collections.ObjectModel;
using GMap.NET.WindowsPresentation;

namespace MapModule.Views
{
    /// <summary>
    /// Interaction logic for MapView.xaml
    /// </summary>
    public partial class MapView : UserControl, IMapView
    {
        public PointLatLng CurrentMouseLatLng;

        public MapView()
        {
            InitializeComponent();

            MainMap.MapProvider = GMap.NET.MapProviders.BingHybridMapProvider.Instance;
            MainMap.Manager.Mode = AccessMode.ServerAndCache;            
            MainMap.DragButton = MouseButton.Left;

            // Set intitial view
            MainMap.Position = new PointLatLng(1.352, 103.811);
            MainMap.Zoom = 11;

            Debug.WriteLine("MapView Initialized");
        }

        private void MainMap_MouseMove(object sender, MouseEventArgs e)
        {
            CurrentMouseLatLng = MainMap.FromLocalToLatLng((int)e.GetPosition(MainMap).X, (int)e.GetPosition(MainMap).Y);

            if (DataContext != null && DataContext is IMapViewModel)
            {
                ((IMapViewModel)DataContext).UpdateCurrentMouseLatLng(CurrentMouseLatLng);
            }
        }

        void IMapView.SetModel(IMapViewModel model)
        {
            DataContext = model;
        }

        void IMapView.UpdateMapMarkers(ObservableCollection<CustomMapMarker> markers)
        {
            // TODO: Assert that MainMap.Markers.Count == markers.Count

            if (MainMap.Markers.Count != markers.Count)
            {
                MainMap.Markers.Clear();

                for (int i = 0; i < markers.Count; i++)
                {
                    MainMap.Markers.Add(markers[i]);
                }
                return;
            }

            // TODO: Check if is it more efficent to create new markers or change the properties of each marker.

            for (int i = 0; i < MainMap.Markers.Count; i++)
            {
                MainMap.Markers[i].Position = new PointLatLng(markers[i].Position.Lat, markers[i].Position.Lng);
            }
        }
    }
}
