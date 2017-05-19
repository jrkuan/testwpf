namespace MinecraftModule.Services
{
    public sealed class Core
    {
        private static Core instance;

        private static object threadlock = new object();

        private Core()
        {
        }

        public static Core Instance
        {
            get
            {
                lock (threadlock)
                {
                    if (instance == null)
                    {
                        return instance = new Core();
                    }
                    else
                    {
                        return instance;
                    }
                }          
            }
        }



    }
}
