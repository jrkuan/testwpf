using MinecraftModule.Services;
using Prism.Modularity;
using Prism.Regions;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
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

            MyDebug.WriteLine("MinecraftModule intialized");
        }

        /// <summary>
        /// Registers a region name with its associated view type in the region view registry
        /// </summary>
        void IModule.Initialize()
        {
            ObservableCollection<Drone> Drones = new ObservableCollection<Drone>();
            Drones.Add(new MavlinkDrone() { Param1 = "COM10", Param2 = 57600, ConnType = DroneModel.ConnectionType.SERIAL});
            //Drones[0].Connect();

            Drones[0].Arm();
        }
    }
}