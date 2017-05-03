using MapModule.Interfaces;
using System;
using MapModule.Views;
using System.Collections.ObjectModel;
using Prism.Mvvm;
using Prism.Commands;

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
            _mapMarkers = _model.GetMarkers();

            _view.SetModel(this);
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

        private DelegateCommand _mapMouseDoubleLefClickCommand;
        public DelegateCommand MapMouseDoubleLeftClickCommand
        {
            get
            {
                return _mapMouseDoubleLefClickCommand = new DelegateCommand(MapMouseDoubleLeftClick);
            }
        }

        public IMapView View
        {
            get
            {
                return _view;
            }
        }

        private void MapMouseDoubleLeftClick()
        {
            
        }
    }
}
