using ConfigModule.Interfaces;
using MapModule.Interfaces;
using Microsoft.Practices.Unity;
using MinecraftModule.Interfaces;
using Prism.Commands;
using Prism.Events;
using Prism.Regions;

namespace ConfigModule.ViewModels
{
    public class ConfigViewModel : IConfigViewModel
    {
        private readonly IConfigModel _model;
        private readonly IConfigView _view;
        private readonly IEventAggregator _eventAggregator;
        private readonly IRegionManager _regionManager;
        private readonly IUnityContainer _container;

        public ConfigViewModel(IConfigView view, IConfigModel model, IEventAggregator eventAggregator, IRegionManager regionManager, IUnityContainer container)
        {
            _view = view;
            _model = model;
            _eventAggregator = eventAggregator;
            _regionManager = regionManager;
            _container = container;

            _view.SetModel(this);
        }

        public IConfigView View
        {
            get
            {
                return _view;
            }
        }

        private DelegateCommand _drone0ConnectCommand;
        public DelegateCommand Drone0ConnectCommand
        {
            get
            {
                return _drone0ConnectCommand = new DelegateCommand(Drone0Connect);
            }
        }

        private DelegateCommand _drone1ConnectCommand;
        public DelegateCommand Drone1ConnectCommand
        {
            get
            {
                return _drone1ConnectCommand = new DelegateCommand(Drone1Connect);
            }
        }

        private void SaveConfig()
        {

        }

        private void Drone0Connect()
        {
            IMinecraft minecraft = _container.Resolve<IMinecraft>();
            minecraft.Connect();
        }

        private void Drone1Connect()
        {

        }
    }
}
