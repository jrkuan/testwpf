using GMap.NET;
using MapModule.Interfaces;
using MapModule.Views;
using Prism.Commands;
using System.Diagnostics;
using System.Windows.Controls;
using System.Windows.Input;
using System;
using System.Collections.ObjectModel;

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

            LatitudeLabel.Text = CurrentMouseLatLng.Lat.ToString("N7");
            LongitudeLabel.Text = CurrentMouseLatLng.Lng.ToString("N7");
        }

        

        void IMapView.SetModel(IMapViewModel model)
        {
            DataContext = model;
        }

    }
}
