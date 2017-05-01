using System;
using Prism;
using Prism.Modularity;
using Prism.Regions;

namespace HelloWorldModule
{
    public class StatusModule : IModule
    {
        private readonly IRegionManager regionManager;

        public StatusModule(IRegionManager regionManager)
        {
            this.regionManager = regionManager;
        }

        /// <summary>
        /// Registers a region name with its associated view type in the region view registry
        /// </summary>
        void IModule.Initialize()
        {
            regionManager.RegisterViewWithRegion("StatusRegion", typeof(Views.HelloWorldView));

        }
    }
}
