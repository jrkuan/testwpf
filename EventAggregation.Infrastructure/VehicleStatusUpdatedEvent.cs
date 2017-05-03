using Prism.Events;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace EventAggregation.Infrastructure
{
    public class VehicleStatusUpdatedEvent : PubSubEvent<VehicleStatus>
    {
    }
}
