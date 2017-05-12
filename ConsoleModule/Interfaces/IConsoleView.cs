using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ConsoleModule.Interfaces
{
    public interface IConsoleView
    {
        void SetModel(IConsoleViewModel model);
    }
}
