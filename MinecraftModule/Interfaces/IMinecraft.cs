using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MinecraftModule.Interfaces
{
    public interface IMinecraft
    {
        /** GCS Methods **/
        void Connect();
        void SelectDrone();

        /** Drone Methods **/
        void Arm();
        void Disarm();
        void Nudge();
        void FlyHere();
        void Land();
        void FollowMe();

        /** Misc Methods **/
        void Custom1();
        void Custom2();
    }
}
