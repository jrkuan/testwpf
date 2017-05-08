using EventAggregation.Infrastructure;
using MinecraftModule.Services;
using Prism.Events;
using StatusModule.Interfaces;

namespace StatusModule.ViewModels
{
    public class StatusViewModel : IStatusViewModel
    {

        private readonly IStatusModel _model;
        private readonly IStatusView _view;
        private readonly IEventAggregator _eventAggregator;

        //private SubscriptionToken subscriptionToken;

        public StatusViewModel(IStatusView view, IStatusModel model, IEventAggregator eventAggregator)
        {
            _view = view;
            _model = model;
            _eventAggregator = eventAggregator;

          SubscriptionToken subscriptionToken = this._eventAggregator.GetEvent<VehicleStatusUpdatedEvent>().Subscribe(VehicleStatusUpdatedEvent_Handle, true);
        }

        IStatusView IStatusViewModel.View
        {
            get
            {
                return _view;
            }
        }

        private void VehicleStatusUpdatedEvent_Handle(object vehStatus)
        {
            MyDebug.WriteLine("StatusModule vehicle status updated");
        }
    }
}
