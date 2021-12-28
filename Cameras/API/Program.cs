// A wrapper C# program for the CamerasAPI 
// Testing the connection to the DLL and its functionality
//
// First created by Chaim Dryzun at 21/11/2021
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using static System.Net.Mime.MediaTypeNames;

namespace pointar_research_api.SkillReal_API_Wrapper
{
    class CamerasAPI
    {

#if UNITY_STANDALONE
    private const string CameraManagmentDllName = @"CamerasManagement_DLL";
    private const string CameraInterfacedllName = @"CameraInterfaceDLL";
#elif DEBUG
        private const string CameraManagmentDllName = @"..\..\..\DLLs\Debug\CamerasManagement_DLL_d.dll";
        private const string CameraInterfacedllName = @"..\..\..\DLLs\Debug\CameraInterfaceDLL_d.dll";
#else
        private const string CameraManagmentDllName = @"..\..\..\DLLs\CamerasManagement_DLL.dll";
        private const string CameraInterfacedllName = @"..\..\..\DLLs\CameraInterfaceDLL.dll";
#endif
        [DllImport(CameraManagmentDllName, EntryPoint =
"CamerasNum", CallingConvention = CallingConvention.StdCall)]
        public static extern int CamerasNum();
        
        [DllImport(CameraInterfacedllName, EntryPoint =
"CreateCamera", CallingConvention = CallingConvention.StdCall)]
        public static extern bool CreateCamera(int CameraType);

        static void Main(string[] args)
        {
            Console.WriteLine("Program end - press any to key to end");
            Console.ReadKey();
        }
    }
}
