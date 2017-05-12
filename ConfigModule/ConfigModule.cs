using ConfigModule.Interfaces;
using ConfigModule.Services;
using ConfigModule.ViewModels;
using ConfigModule.Views;
using Microsoft.Practices.Unity;
using Prism.Modularity;
using Prism.Regions;

namespace ConfigModule
{
    public class ConfigModule : IModule
    {
        private readonly IRegionManager regionManager;
        private readonly IUnityContainer container;

        public ConfigModule(IUnityContainer container, IRegionManager regionManager)
        {
            this.container = container;
            this.regionManager = regionManager;
        }

        void IModule.Initialize()
        {
            container.RegisterType<IConfigModel, ConfigModel>();
            container.RegisterType<IConfigViewModel, ConfigViewModel>();
            container.RegisterType<IConfigView, ConfigView>();

            IConfigViewModel view = container.Resolve<IConfigViewModel>();

            regionManager.Regions["ConfigRegion"].Add(view.View);
        }
    }
}
