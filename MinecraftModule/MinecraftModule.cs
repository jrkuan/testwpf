using Prism.Modularity;
using Prism.Regions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MinecraftModule
{
    public class MinecraftModule : IModule
    {
        private readonly IRegionManager regionManager;

        public MinecraftModule(IRegionManager regionManager)
        {
            this.regionManager = regionManager;
        }

        /// <summary>
        /// Registers a region name with its associated view type in the region view registry
        /// </summary>
        void IModule.Initialize()
        {
                

        }
    }
}