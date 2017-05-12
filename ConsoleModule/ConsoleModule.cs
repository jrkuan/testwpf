using ConsoleModule;
using ConsoleModule.Interfaces;
using ConsoleModule.ViewModels;
using ConsoleModule.Views;
using Microsoft.Practices.Unity;
using Prism.Modularity;
using Prism.Regions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ConsoleModule
{
    public class ConsoleModule : IModule
    {
        private readonly IRegionManager regionManager;
        private readonly IUnityContainer container;

        public ConsoleModule(IUnityContainer container, IRegionManager regionManager)
        {
            this.container = container;
            this.regionManager = regionManager;
        }

        void IModule.Initialize()
        {
            container.RegisterType<IConsoleView, ConsoleView>();
            container.RegisterType<IConsoleViewModel, ConsoleViewModel>();

            IConsoleViewModel view = container.Resolve<IConsoleViewModel>();

            regionManager.Regions["ConsoleRegion"].Add(view.View);
        }
    }
}
