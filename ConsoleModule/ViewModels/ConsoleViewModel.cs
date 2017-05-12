using ConsoleModule.Interfaces;
using Prism.Events;
using Prism.Mvvm;
using Prism.Regions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ConsoleModule.ViewModels
{
    public class ConsoleViewModel : BindableBase, IConsoleViewModel
    {
        //private readonly IConsoleModel _model;
        private readonly IConsoleView _view;
        private readonly IEventAggregator _eventAggregator;
        private readonly IRegionManager _regionManager;

        public ConsoleViewModel(IConsoleView view, IEventAggregator eventAggregator, IRegionManager regionManager)
        {
            _view = view;
            _eventAggregator = eventAggregator;
            _regionManager = regionManager;

            _view.SetModel(this);
        }

        public IConsoleView View
        {
            get
            {
                return _view;
            }
        }
    }
}
