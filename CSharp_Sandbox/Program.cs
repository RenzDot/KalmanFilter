using System;
using System.Collections.Generic;
using System.Linq;

namespace CSharp_Sandbox
{
    class Program
    {
        static void Main(string[] args) {
            var Wm = new List<double>(new double[10]);
            foreach (var s in Wm) {
                Console.WriteLine(s.ToString());
            }
        }
    }
}
