using MapModule.Interfaces;
using System;
using GMap.NET;
using MapModule.Views;
using System.Collections.ObjectModel;
using Prism.Mvvm;
using Prism.Commands;
using System.Collections.Specialized;

namespace MapModule.ViewModels
{
    public class MapViewModel : BindableBase, IMapViewModel
    {
        private readonly IMapModel _model;
        private readonly IMapView _view;
        private ObservableCollection<CustomMapMarker> _mapMarkers;
        
        public MapViewModel(IMapView view, IMapModel model)
        {
            _view = view;
            _model = model;

            _mapMarkers = new ObservableCollection<CustomMapMarker>();
            //_mapMarkers = _model.GetMarkers();

            _view.SetModel(this);

            MapMarkers.CollectionChanged += MapMarkers_CollectionChanged;
        }

        /// <summary>
        /// 
        /// </summary>
        private void MapMarkers_CollectionChanged(object sender, NotifyCollectionChangedEventArgs e)
        {
            _view.UpdateMapMarkers(MapMarkers);
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

        private DelegateCommand _mapMouseDoubleLeftClickCommand;
        public DelegateCommand MapMouseDoubleLeftClickCommand
        {
            get
            {
                return _mapMouseDoubleLeftClickCommand = new DelegateCommand(MapMouseDoubleLeftClick);
            }
        }

        //private DelegateCommand _mapMouseMoveCommand;
        //public DelegateCommand MapMouseMoveCommand
        //{
        //    get
        //    {
        //        //return _mapMouseMoveCommand = new DelegateCommand(MapMouseMove);
        //    }
        //}

        public IMapView View
        {
            get
            {
                return _view;
            }
        }

        private void MapMouseDoubleLeftClick()
        {
            MapMarkers.Add(new CustomMapMarker(CustomMapMarker.TagType.Waypoint, new PointLatLng(CurrentMouseLatitude, CurrentMouseLongitude), "waypoint", 0) { Offset = new System.Windows.Point(-50,-50)});
        }

        void IMapViewModel.UpdateCurrentMouseLatLng(PointLatLng currentMouseLatLng)
        {
            CurrentMouseLatitude = currentMouseLatLng.Lat;
            CurrentMouseLongitude = currentMouseLatLng.Lng;
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

    }
}
