using StatusModule.Interfaces;
using System.Windows.Controls;

namespace StatusModule.Views
{
    /// <summary>
    /// Interaction logic for StatusView.xaml
    /// </summary>
    public partial class StatusView : UserControl, IStatusView
    {
        public StatusView()
        {
            InitializeComponent();
        }
    }
}
