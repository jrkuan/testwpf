using System;
using Prism;
using StatusModule.Services;
using StatusModule.Views;
using StatusModule.ViewModels;
using StatusModule.Interfaces;
using Prism.Modularity;
using Prism.Regions;
using Microsoft.Practices.Unity;

namespace StatusModule
{
    public class StatusModel : IModule
    {
        private readonly IRegionManager regionManager;
        private readonly IUnityContainer container;

        public StatusModel(IUnityContainer container, IRegionManager regionManager)
        {
            this.regionManager = regionManager;
            this.container = container;
        }

        /// <summary>
        /// Registers a region name with its associated view type in the region view registry
        /// </summary>
        void IModule.Initialize()
        {

            container.RegisterType<IStatusModel, Services.StatusModel>();
            container.RegisterType<IStatusViewModel, ViewModels.StatusViewModel>();
            container.RegisterType<IStatusView, StatusView>();

            IStatusViewModel view = container.Resolve<IStatusViewModel>();

            //regionManager.RegisterViewWithRegion("StatusRegion", typeof(Views.StatusView));
            regionManager.Regions["StatusRegion"].Add(view.View);

        }
    }
}
