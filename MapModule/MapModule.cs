using Prism.Modularity;
using Prism.Regions;

namespace MapModule
{
    public class MapModule : IModule
    {
        private readonly IRegionManager regionManager;

        public MapModule(IRegionManager regionManager)
        {
            this.regionManager = regionManager;
        }

        void IModule.Initialize()
        {
            regionManager.RegisterViewWithRegion("MainRegion", typeof(Views.MapView));
        }
    }
}
