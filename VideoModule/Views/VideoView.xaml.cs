using System.Windows;
using System.Windows.Controls;

namespace VideoModule.Views
{
    /// <summary>
    /// Interaction logic for VideoView.xaml
    /// </summary>
    public partial class VideoView : UserControl
    {
        //private bool _mouseDownToolBar = false;
        //private Point _dragOffSet;

        public VideoView()
        {
            InitializeComponent();
        }

        //private void OnToolbarClicked(object sender, MouseButtonEventArgs e)
        //{
        //    _mouseDownToolBar = true;
        //    _dragOffSet = e.GetPosition(DroneVideoPanel);
        //    DroneVideoPanel.CaptureMouse();
        //}

        //private void OnToolbarReleased(object sender, MouseButtonEventArgs e)
        //{
        //    _mouseDownToolBar = false;
        //    DroneVideoPanel.ReleaseMouseCapture();
        //}

        //private void OnToolbarMoving(object sender, MouseEventArgs e)
        //{
        //    if (_mouseDownToolBar)
        //    {
        //        // we want to move it based on the position of the mouse
        //        MoveUserControl(e);
        //    }
        //}

        //private void MoveUserControl(MouseEventArgs e)
        //{
        //    Point mousePos = e.GetPosition(DroneVideoPanel);
        //    Double newX = LocalTranslateTransform.X + (mousePos.X - _dragOffSet.X);
        //    Double newY = LocalTranslateTransform.Y + (mousePos.Y - _dragOffSet.Y);
        //    LocalTranslateTransform.X = newX;
        //    LocalTranslateTransform.Y = newY;
        //}


    }
}
