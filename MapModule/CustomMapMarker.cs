using GMap.NET.WindowsPresentation;
using GMap.NET;
using MultiDroneGCS1.Display.CustomControl;
using Prism.Commands;

namespace MapModule.Views
{
    public class CustomMapMarker : GMapMarker
    {
        #region Enums

        public enum TagType
        {
            Drone,
            Waypoint,
            RedForce,
            BlueForce
        }

        #endregion Enums

        #region Properties

        public int droneID { get; private set; }
        public string Description { get; set; }
        public GMapMarker GMapMark { get; set; }

        #endregion Properties

        #region Constructors

        public CustomMapMarker(TagType tag, PointLatLng pos, string description, int index) : base(pos)
        {
            Tag = tag;
            Description = description;
            droneID = index;

            switch (tag)
            {
                case TagType.Drone:
                    Shape = new DroneMarkerUserControl();
                    Shape.IsHitTestVisible = true;

                    ((DroneMarkerUserControl)Shape).UAVNameLabel.Content = droneID.ToString();
                    break;
                case TagType.Waypoint:
                    Shape = new WaypointUserControl();

                    ((WaypointUserControl)Shape).IndexLabel.Content = droneID.ToString();
                    break;
            }                    
        }

        private DelegateCommand _mouseClickCommand;
        public DelegateCommand MouseClickCommand
        {
            get
            {
                return _mouseClickCommand = new DelegateCommand(MouseClick);
            }
        }

        private void MouseClick()
        {
        }
        #endregion Constructors
    }
}
