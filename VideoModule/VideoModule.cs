using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Prism.Modularity;
using Prism.Regions;

namespace VideoModule
{
    public class VideoModule : IModule
    {
        private readonly IRegionManager regionManager;

        public VideoModule(IRegionManager regionManager)
        {
            this.regionManager = regionManager;
        }

        void IModule.Initialize()
        {
            return;
        }
    }
}
