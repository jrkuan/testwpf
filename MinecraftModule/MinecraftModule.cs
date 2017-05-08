using EventAggregation.Infrastructure;
using MinecraftModule.Services;
using Prism.Events;
using Prism.Modularity;
using Prism.Regions;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace MinecraftModule
{
    public class MinecraftModule : IModule
    {
        private readonly IRegionManager regionManager;
        private readonly IEventAggregator eventAggregator;

        private BackgroundWorker BGW = new BackgroundWorker();

        ObservableCollection<Drone> Drones = new ObservableCollection<Drone>();

        public MinecraftModule(IRegionManager regionManager, IEventAggregator eventAggregator)
        {
            this.regionManager = regionManager;
            this.eventAggregator = eventAggregator;

            BGW.DoWork += BGW_DoWork;
            BGW.RunWorkerAsync();

            MyDebug.WriteLine("MinecraftModule intialized");
        }

        /// <summary>
        /// Registers a region name with its associated view type in the region view registry
        /// </summary>
        void IModule.Initialize()
        {            
            Drones.Add(new MavlinkDrone() { Param1 = "COM10", Param2 = 57600, ConnType = DroneModel.ConnectionType.SERIAL});
            Drones[0].Connect();

            Drones[0].Arm();
        }

        private void BGW_DoWork(object sender, DoWorkEventArgs e)
        {
            VehicleStatus vehStatus = new VehicleStatus();
            vehStatus.latitude = 111;
            vehStatus.longitude = 222;

            while (true)
            {
                eventAggregator.GetEvent<VehicleStatusUpdatedEvent>().Publish(vehStatus);
                //MyDebug.WriteLine("tick");
                Thread.Sleep(1000);

            }
        }
    }
}