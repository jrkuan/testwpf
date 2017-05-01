using System.Windows;
using Prism.Unity;
using Prism.Modularity;
using HelloWorldModule;

namespace HelloWorld.Desktop
{
    class Bootstrapper : UnityBootstrapper
    {
        /// <summary>
        /// Create an instance of the shell window and return it
        /// </summary>
        /// <returns></returns>
        protected override DependencyObject CreateShell()
        {
            return new Shell();
        }

        /// <summary>
        /// Display the shell to the user
        /// </summary>
        protected override void InitializeShell()
        {
            base.InitializeShell();

            Application.Current.MainWindow = (Window)this.Shell;
            Application.Current.MainWindow.Show();
        }

        /// <summary>
        /// Populate the module catalog with modules
        /// </summary>
        protected override void ConfigureModuleCatalog()
        {
            base.ConfigureModuleCatalog();
            ModuleCatalog moduleCatalog = (ModuleCatalog)this.ModuleCatalog;
            moduleCatalog.AddModule(typeof(StatusModule));
        }

    }
}
