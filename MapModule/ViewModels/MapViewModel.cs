using MapModule.Interfaces;
using System;
using GMap.NET;
using MapModule.Views;
using System.Collections.ObjectModel;
using Prism.Mvvm;
using Prism.Commands;
using System.Collections.Specialized;
using System.Windows.Controls.Primitives;
using System.Threading;
using MapModule.Services;
using System.Windows;
using System.ComponentModel;
using Prism.Events;
using EventAggregation.Infrastructure;
using MinecraftModule.Services;
using Prism.Regions;
using Microsoft.Practices.Unity;
using MinecraftModule.Interfaces;
using System.Diagnostics;

namespace MapModule.ViewModels
{
    public class MapViewModel : BindableBase, IMapViewModel
    {
        private readonly IMapModel _model;
        private readonly IMapView _view;
        private readonly IEventAggregator _eventAggregator;
        private readonly IRegionManager _regionManager;
        private readonly IUnityContainer _container;

        private int _selectedDrone = 0;

        private ObservableCollection<CustomMapMarker> _mapMarkers;
        public ObservableCollection<WaypointGridItem> waypointGridItems { get; }

        public MapViewModel(IMapView view, IMapModel model, IEventAggregator eventAggregator, IRegionManager regionManager, IUnityContainer container)
        {
            _view = view;
            _model = model;
            _eventAggregator = eventAggregator;
            _regionManager = regionManager;
            _container = container;

            _mapMarkers = new ObservableCollection<CustomMapMarker>();
            waypointGridItems = new ObservableCollection<WaypointGridItem>();

            SubscriptionToken subscriptionToken = this._eventAggregator.GetEvent<VehicleStatusUpdatedEvent>().Subscribe(EventTick);

            _view.SetModel(this);

            MapMarkers.CollectionChanged += MapMarkers_CollectionChanged;

            MapMarkers.Add(new CustomMapMarker(CustomMapMarker.TagType.Drone, new PointLatLng(1.3113, 103.72947), "drone", 0) { });
            MapMarkers.Add(new CustomMapMarker(CustomMapMarker.TagType.Drone, new PointLatLng(1.3132, 103.73947), "drone", 1) { });
            waypointGridItems.CollectionChanged += WaypointGridItems_CollectionChanged;
        }

        /// <summary>
        /// 
        /// </summary>
        private void MapMarkers_CollectionChanged(object sender, NotifyCollectionChangedEventArgs e)
        {
            _view.UpdateMapMarkers(MapMarkers);
        }

        private void WaypointGridItems_CollectionChanged(object sender, NotifyCollectionChangedEventArgs e)
        {
            CustomMapMarker cmm;

            if (_selectedDrone > -1)
            {
                if (GetWaypointMarkerCount() > waypointGridItems.Count)
                {   // If marker count and griditem count are out of sync
                    RemoveWaypointMarkers();

                    int newIndex = 0;

                    foreach (WaypointGridItem wgi in waypointGridItems)
                    {
                        wgi.Index = newIndex++;
                    }
                }

                foreach (WaypointGridItem wgi in waypointGridItems)
                {
                    cmm = new CustomMapMarker(CustomMapMarker.TagType.Waypoint, new PointLatLng(wgi.Lat, wgi.Lng), _selectedDrone.ToString(), wgi.Index) { Offset = new Point(-50, -50) };
                    (cmm.Shape as WaypointUserControl).AltitudeLabel.Content = wgi.Alt;
                    (cmm.Shape as WaypointUserControl).DwellTimeLabel.Content = wgi.Delay;
                    MapMarkers.Add(cmm);
                }
            }
            else
            {
                // Should not come here since WaypointGridItems cannot be added if there is no drone selected
                ClearWaypoints();
            }
            _view.UpdateMapMarkers(MapMarkers);
        }

        private void WaypointGridItem_PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            RemoveWaypointMarkers();
            WaypointGridItems_CollectionChanged(sender, null);
        }

        private void EventTick(object payload)
        {
            MyDebug.WriteLine("MapModule vehicle status updated");
        }

        public ObservableCollection<CustomMapMarker> MapMarkers
        {
            get
            {
                return _mapMarkers;
            }
            set
            {
                SetProperty(ref _mapMarkers, value);
            }
        }

        #region DelegateCommand

        private DelegateCommand _mapMouseDoubleLeftClickCommand;
        public DelegateCommand MapMouseDoubleLeftClickCommand
        {
            get
            {
                return _mapMouseDoubleLeftClickCommand = new DelegateCommand(MapMouseDoubleLeftClick, MapMouseDoubleLeftClickCanExecute);
            }
        }

        private DelegateCommand _clearWaypointsCommand;
        public DelegateCommand ClearWaypointsCommand
        {
            get
            {
                return _clearWaypointsCommand = new DelegateCommand(ClearWaypoints);
            }
        }

        private DelegateCommand _startWaypointMissionCommand;
        public DelegateCommand StartWaypointMissionCommand
        {
            get
            {
                return _startWaypointMissionCommand = new DelegateCommand(StartWaypointMission);
            }
        }

        private DelegateCommand _stopWaypointMissionCommand;
        public DelegateCommand StopWaypointMissionCommand
        {
            get
            {
                return _stopWaypointMissionCommand = new DelegateCommand(StopWaypointMission);
            }
        }

        private DelegateCommand _downloadMissionCommand;
        public DelegateCommand DownloadMisisonCommand
        {
            get
            {
                return _downloadMissionCommand = new DelegateCommand(DownloadMission);
            }
        }

        private DelegateCommand _uploadMissionCommand;
        public DelegateCommand UploadMissionCommand
        {
            get
            {
                return _uploadMissionCommand = new DelegateCommand(UploadMission);
            }
        }

        private DelegateCommand _armDroneCommand;
        public DelegateCommand ArmDroneCommand
        {
            get
            {
                return _armDroneCommand = new DelegateCommand(ArmDrone);
            }
        }

        private DelegateCommand _disarmDroneCommand;
        public DelegateCommand DisarmDroneCommand
        {
            get
            {
                return _disarmDroneCommand = new DelegateCommand(DisarmDrone);
            }
        }

        #endregion DelegateCommand

        public IMapView View
        {
            get
            {
                return _view;
            }
        }        

        void IMapViewModel.UpdateCurrentMouseLatLng(PointLatLng currentMouseLatLng)
        {
            CurrentMouseLatitude = currentMouseLatLng.Lat;
            CurrentMouseLongitude = currentMouseLatLng.Lng;
        }

        private void MapMouseDoubleLeftClick()
        {
            WaypointGridItem wgi = new WaypointGridItem() { Index = waypointGridItems.Count, Lat = CurrentMouseLatitude, Lng = CurrentMouseLongitude };
            wgi.PropertyChanged += WaypointGridItem_PropertyChanged;
            waypointGridItems.Add(wgi);
        }

        private bool MapMouseDoubleLeftClickCanExecute()
        {
            return ExpanderIsExpanded;
        }

        private void ClearWaypoints()
        {
            waypointGridItems.Clear();

            RemoveWaypointMarkers();
        }

        private void RemoveWaypointMarkers()
        {
            ObservableCollection<CustomMapMarker> newCollection = new ObservableCollection<CustomMapMarker>();
            foreach (CustomMapMarker cmm in MapMarkers)
            {
                if ((CustomMapMarker.TagType)cmm.Tag != CustomMapMarker.TagType.Waypoint)
                {
                    newCollection.Add(cmm);
                }
            }
            MapMarkers.Clear();

            foreach (CustomMapMarker cmm in newCollection)
            {
                MapMarkers.Add(cmm);
            }
        }

        private int GetWaypointMarkerCount()
        {
            int count = 0;
            foreach (CustomMapMarker cmm in MapMarkers)
            {
                if ((CustomMapMarker.TagType)cmm.Tag == CustomMapMarker.TagType.Waypoint)
                {
                    count++;
                }
            }
            return count;
        }

        private void StopWaypointMission()
        {
        }

        private void StartWaypointMission()
        {

        }

        private void UploadMission()
        {

        }

        private void DownloadMission()
        {

        }

        private void ArmDrone()
        {
            if (_selectedDrone == -1)
            {
                return;
            }
            IMinecraft minecraft = _container.Resolve<IMinecraft>();
            minecraft.Arm();
        }

        private void DisarmDrone()
        {

        }

        private double _currentMouseLatitude;
        public double CurrentMouseLatitude
        {
            get
            {
                return _currentMouseLatitude;
            }
            set
            {
                SetProperty(ref _currentMouseLatitude, value);
            }
        }

        private double _currentMouseLongitude;
        public double CurrentMouseLongitude
        {
            get
            {
                return _currentMouseLongitude;
            }
            set
            {
                SetProperty(ref _currentMouseLongitude, value);
            }
        }

        private bool _expanderIsExpanded;
        public bool ExpanderIsExpanded
        {
            get
            {
                return _expanderIsExpanded;
            }
            set
            {
                SetProperty(ref _expanderIsExpanded, value);
            }
        }


    }
}
