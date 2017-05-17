using Microsoft.Practices.Unity;
using MinecraftModule.Interfaces;
using Prism.Events;
using Prism.Regions;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace MinecraftModule.Services
{
    public class Minecraft : IMinecraft
    {
        private readonly IEventAggregator _eventAggregator;
        private readonly IRegionManager _regionManager;
        private readonly IUnityContainer _container;

        ObservableCollection<Drone> Drones = new ObservableCollection<Drone>();

        private int _selectedDrone = 0;

        public Minecraft(IEventAggregator eventAggregator, IRegionManager regionManager, IUnityContainer container)
        {
            this._eventAggregator = eventAggregator;
            this._regionManager = regionManager;
            this._container = container;

            Connect();
            //Drones.CollectionChanged += 
        }

        public void UpdateSelectedDrone(int selectedDrone)
        {
            _selectedDrone = selectedDrone;
        }

        public void Arm()
        {
            ThreadPool.QueueUserWorkItem(delegate { ArmDrone(); });
        }

        private void ArmDrone()
        {
            Drones[_selectedDrone].Arm();
        }


        public void Connect()
        {
            Drones.Add(new MavlinkDrone() {Param1 = "COM10", Param2 = 57600, ConnType = DroneModel.ConnectionType.SERIAL, AutoReconnect = false });
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
            ThreadPool.QueueUserWorkItem(delegate { DisarmDrone(); });
        }

        private void DisarmDrone()
        {

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
