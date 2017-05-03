using Prism.Events;
using EventAggregation.Infrastructure;

namespace StatusModule.ViewModels
{
    public class StatusViewPresenter
    {
        //private IAddFundView _view;
        private IEventAggregator eventAggregator;
        private SubscriptionToken subscriptionToken;

        public StatusViewPresenter(IEventAggregator eventAggregator)
        {
            this.eventAggregator = eventAggregator;
        }

        public void VehicleStatusUpdatedEventHandler(VehicleStatus vehicleStatus)
        {
            //Debug.Assert(View != null);

        }

        //public IActivityView {get;set;}

        private float _latitudeValue;
        public float LatitudeValue
        {
            get
            {
                return _latitudeValue;
            }
            set
            {
                _latitudeValue = value;
            }
        }

        private float _pitchValue;
        public float PitchValue
        {
            get
            {
                return _pitchValue;
            }
            set
            {
                _pitchValue = value;
            }
        }


    }
}
