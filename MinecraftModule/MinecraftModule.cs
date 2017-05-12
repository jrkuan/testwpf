using EventAggregation.Infrastructure;
using Microsoft.Practices.Unity;
using MinecraftModule.Interfaces;
using MinecraftModule.Services;
using Prism.Events;
using Prism.Modularity;
using Prism.Regions;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Threading;

namespace MinecraftModule
{
    public class MinecraftModule : IModule
    {
        private readonly IEventAggregator _eventAggregator;
        private readonly IRegionManager _regionManager;
        private readonly IUnityContainer _container;

        private BackgroundWorker BGW = new BackgroundWorker();

        ObservableCollection<Drone> Drones = new ObservableCollection<Drone>();

        public MinecraftModule(IRegionManager regionManager, IEventAggregator eventAggregator, IUnityContainer container)
        {
            this._regionManager = regionManager;
            this._eventAggregator = eventAggregator;
            this._container = container;

            MyDebug.WriteLine("MinecraftModule intialized");
        }

        /// <summary>
        /// Registers a region name with its associated view type in the region view registry
        /// </summary>
        void IModule.Initialize()
        {
            _container.RegisterType<IMinecraft, Minecraft>();
        }
        
    }
}