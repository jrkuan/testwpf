using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ConfigModule.Interfaces
{
    public interface IConfigViewModel
    {
        IConfigView View { get; }
    }
}
