using GMap.NET;
using System.Diagnostics;
using System.Windows.Controls;
using System.Windows.Input;

namespace MapModule.Views
{
    /// <summary>
    /// Interaction logic for MapView.xaml
    /// </summary>
    public partial class MapView : UserControl
    {

        PointLatLng CurrentMouseLatLng;

        public MapView()
        {
            InitializeComponent();

            MainMap.MapProvider = GMap.NET.MapProviders.BingHybridMapProvider.Instance;
            MainMap.Manager.Mode = AccessMode.ServerAndCache;            
            MainMap.DragButton = MouseButton.Left;

            Debug.WriteLine("MapView Initialized");
        }

        private void MainMap_MouseMove(object sender, MouseEventArgs e)
        {
            CurrentMouseLatLng = MainMap.FromLocalToLatLng((int)e.GetPosition(MainMap).X, (int)e.GetPosition(MainMap).Y);

        }
    }
}
