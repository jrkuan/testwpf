using Microsoft.Practices.Unity;
using MinecraftModule.Interfaces;
using Prism.Events;
using Prism.Regions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MinecraftModule.Services
{
    public class Minecraft : IMinecraft
    {
        private readonly IEventAggregator _eventAggregator;
        private readonly IRegionManager _regionManager;
        private readonly IUnityContainer _container;

        public Minecraft(IEventAggregator eventAggregator, IRegionManager regionManager, IUnityContainer container)
        {
            this._eventAggregator = eventAggregator;
            this._regionManager = regionManager;
            this._container = container;
        }

        public void Arm()
        {
            
        }

        public void Connect()
        {
            
        }

        public void Custom1()
        {
            throw new NotImplementedException();
        }

        public void Custom2()
        {
            throw new NotImplementedException();
        }

        public void Disarm()
        {
            throw new NotImplementedException();
        }

        public void FlyHere()
        {
            throw new NotImplementedException();
        }

        public void FollowMe()
        {
            throw new NotImplementedException();
        }

        public void Land()
        {
            throw new NotImplementedException();
        }

        public void Nudge()
        {
            throw new NotImplementedException();
        }

        public void SelectDrone()
        {
            throw new NotImplementedException();
        }
    }
}
