using System.Windows;
using Prism.Unity;
using Prism.Modularity;
using Prism.Mvvm;
using MinecraftModule;
using Prism.Events;
using Microsoft.Practices.Unity;
using ConsoleModule;
using ConfigModule;

namespace Minecraft.Desktop
{
    class Bootstrapper : UnityBootstrapper
    {
        public readonly IEventAggregator _eventAggregator = new EventAggregator();
        
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

            Container.RegisterInstance(_eventAggregator);

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
            moduleCatalog.AddModule(typeof(StatusModule.StatusModel));
            moduleCatalog.AddModule(typeof(MapModule.MapModule));
            moduleCatalog.AddModule(typeof(MinecraftModule.MinecraftModule));
            moduleCatalog.AddModule(typeof(ConsoleModule.ConsoleModule));
            moduleCatalog.AddModule(typeof(ConfigModule.ConfigModule));
        }

    }
}
