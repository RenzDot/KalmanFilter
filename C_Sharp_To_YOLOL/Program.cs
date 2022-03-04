using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C_Sharp_To_YOLOL
{
    class Program
    {
        static void Main(string[] args) {
            string C_Sharp = @"
            xm = xk + _v + 0.5 * a * t * t;
            vm = _v + a * t;
            cx = px + pv*(t * t);
            cv = pv;

            kx = cx / (cx + (ex * ex));
            kv = cv / (cv + (ev * ev));

            zx = Pow(Pow(_XX- x1, 2) + Pow(_YY-y1,2) + Pow(_ZZ-z1,2),1/2);
            x1 = _XX; y1 = _YY; z1 = _ZZ;
            xk = xm + kx * (zx - xm);
            _v = vm + kv * (zv - vm);
            zv = _v;

            px = cx * (1 - kx);
            pv = cv * (1 - kv);
            ";

            YOLOL yolol = new YOLOL(C_Sharp);
            Console.Write(yolol.Get()); 
            Console.Read();
        }
    }

    public class YOLOL {
        string yolol = "";
        string line = "";
        string statement = "";
        string[] replacement = new string[] { "_", ":" };
        int maxLength = 70;
        public YOLOL(string C_Sharp) {
            C_Sharp = C_Sharp.Replace(Environment.NewLine, " ");
            C_Sharp = C_Sharp.Replace(" ", "");
            foreach (char letter in C_Sharp) {
                if (letter != ';') {
                    char c = (letter == replacement[0][0]) ? replacement[1][0] : letter;
                    statement += c;
                } else {
                    statement += " ";
                    if (line.Length + statement.Length - 1 < maxLength) {
                        line += statement;
                        statement = ""; 
                    } else {
                        yolol += line + Environment.NewLine;
                        line = "";
                    }
                }
            }

            yolol += line;
        }

        public string Get() {
            return yolol;
        }
    }
}
