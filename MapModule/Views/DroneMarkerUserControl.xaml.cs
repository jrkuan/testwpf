using Prism.Commands;
using Prism.Mvvm;
using System.Windows.Controls;

namespace MapModule.Views
{
    public partial class DroneMarkerUserControl : UserControl
    {
        #region Constructor

        public DroneMarkerUserControl()
        {
            InitializeComponent();
        }

        #endregion

        #region Methods

        

        private void Grid_MouseDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {

            //Toggle drone ring visibility upon pressing
            SelectorRing.Visibility = SelectorRing.Visibility == System.Windows.Visibility.Hidden ? System.Windows.Visibility.Visible : System.Windows.Visibility.Hidden;
        }

        #endregion
    }
}
