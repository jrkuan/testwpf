using MapModule.Interfaces;
using MapModule.Services;
using MapModule.ViewModels;
using MapModule.Views;
using Microsoft.Practices.Unity;
using Prism.Modularity;
using Prism.Regions;

namespace MapModule
{
    public class MapModule : IModule
    {
        private readonly IRegionManager regionManager;
        private readonly IUnityContainer container;

        public MapModule(IUnityContainer container, IRegionManager regionManager)
        {
            this.container = container;
            this.regionManager = regionManager;
        }

        void IModule.Initialize()
        {
            container.RegisterType<IMapModel, MapModel>();
            container.RegisterType<IMapViewModel, MapViewModel>();
            container.RegisterType<IMapView, MapView>();

            IMapViewModel view = container.Resolve<IMapViewModel>();

            //regionManager.RegisterViewWithRegion("MainRegion", typeof(Views.MapView));
            regionManager.Regions["MainRegion"].Add(view.View);
        }
    }
}
