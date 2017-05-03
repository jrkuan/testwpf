using GMap.NET;
using StatusModule.ViewModels;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace StatusModule.Views
{
    /// <summary>
    /// Interaction logic for HelloWorldView.xaml
    /// </summary>
    public partial class StatusView : UserControl
    {
        public StatusView()
        {
            InitializeComponent();
        }

        public StatusView(StatusViewPresenter presenter) : this()
        {

        }
    }
}
