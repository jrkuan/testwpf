using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace MinecraftModule.Services
{
    public static class MyDebug
    {
        [DllImport("Kernel32")]
        public static extern void AllocConsole();

        [DllImport("Kernel32")]
        public static extern void FreeConsole();

        /// <summary>
        /// Writes a new line to the debugger output. Includes caller name and line number.
        /// </summary>
        public static void WriteLine(object message, [CallerFilePath] string filePath = "", [CallerLineNumber] int lineNumber = 0,
    [CallerMemberName] string caller = null)
        {
            Debug.WriteLine($"{message.ToString()} [{caller}/line:{lineNumber}]");
        }

        /// <summary>
        /// Writes a new line to the console.
        /// </summary>
        public static void ConsoleWriteLine(object message)
        {
            AllocConsole();
            Console.WriteLine(message.ToString());
        }

        /// <summary>
        /// Appends a string to the console.
        /// </summary>
        public static void ConsoleWrite(object message)
        {
            AllocConsole();
            Console.Write(message.ToString());
        }
        
        /// <summary>
        /// Appends a string to a text file.
        /// </summary>
        public static void ToFile(string message, string fileName)
        {
            return;
        }

    }
}