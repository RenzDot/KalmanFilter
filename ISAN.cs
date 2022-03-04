using System;

public interface Iisan() {}

public class ISAN2_Mono: Iisan
{
    public double _x, _y, _z, _w;
    public double[] _ax, _ay, _az, _aw;
    string x, y, z;
    double a, b, c, d, t, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s;

    public ISAN2_Mono() {
        z = "station_"; w = z + "hq_imperial_a"; x = z + "proving_grounds";
        y = z + "kingdom_outpost_b"; z += "kingdom_outpost_b_2";
        t = 999999; _t = w;
        e = 207744.954; f = -1550111.534; g = 486779.292; h = -160699.392; i = -46169.175;
        j = -140483.070; k = 103669.797; l = 799664.698; m = -264673.522; n = -4718.935;
        o = -230400.908; p = 345153.593; q = 41051.448; r = -43636.344; s = -1606.533;
    }

    public void Update() {
        _t = x; a = Pow((t - _ax), 2); ar = (a - la) / 4 arr = (ar - lar) / 4 la = a lar = ar u = 7 goto 10



    }

    public void SetData(double[] xx, double[] yy, double[] zz, double[] ww) {
        _ax = xx; _ay = yy; _az = zz; _aw = ww;
    }
}


public class ISAN2 : Iisan
{
    double _X1, _X2, _X3, _X4 = 0;
    double _Y1, _Y2, _Y3, _Y4 = 0;
    double _Z1, _Z2, _Z3, _Z4 = 0;
    double _rst;

    double _XA, _XB, _XC, _XD = 0;
    double _YA, _YB, _YC, _YD = 0;
    double _ZA, _ZB, _ZC, _ZD = 0;
    public double _XE, _X, _YE, _Y, _ZE, _Z = 0;

    string _M1, _M2, _M3, _M4;

    //double A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z;
    //double a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w, x, y, z;

    public ISAN2() {
        Setup();
    }

    public void Setup() {
        string m;
        double _rst, done;

        m = "station_";// goto 2 - done

        _M1 = m + "hq_imperial_a"; _X1 = -9938.401; _Y1 = 4904.714; _Z1 = 0;
        _M2 = m + "hq_kingdom_a"; _X2 = 9894.287; _Y2 = 4904.714; _Z2 = 0;
        _M3 = m + "proving_grounds"; _X3 = 19218.818; _Y3 = -45540.987; _Z3 = 0;
        _M4 = m + "capital_imperial_a"; _X4 = -465.876; _Y4 = 0.236; _Z4 = -801.149;
        _rst = 1; done = 1; //ISAN 2.0_Temp_Setup_0.1a THE COLLECTIVE
    }

    public void InversionEngine() {
        double A, C;
        double e, h, i, m, n, r, t, u, z;
        double da, P1, P2, P3, P4;

        //goto 1 + (_rst == 1);
        A = _X1 - _X2; z = _Y1 - _Y2; u = _Z1 - _Z2; r = _X1 - _X3; e = _Y1 - _Y3; t = _Z1 - _Z3; h = _X1 - _X4;
        i = _Y1 - _Y4; C = _Z1 - _Z4; da = A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h; n = -0.5;
        z = _Y2; u = _Z2; e = _Y3; t = _Z3; i = _Y4; C = _Z4; m = 9223372036854775.807;

        _XA = m; A = n; r = n; h = n; _XA = da / (A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h);
        _YA = m; z = _X2; e = _X3; i = _X4; _YA = -da / (A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h);
        _ZA = m; u = _Y2; t = _Y3; C = _Y4; _ZA = da / (A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h);
        _ZB = m; z = _X1; u = _Y1; _ZB = -da / (A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h);
        _YB = m; u = _Z1; t = _Z3; C = _Z4; _YB = da / (A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h);
        _XB = m; z = _Y1; e = _Y3; i = _Y4; _XB = -da / (A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h);
        e = _Y2; t = _Z2;
        _XC = m; A = n; r = n; h = n; _XC = da / (A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h);
        _YC = m; z = _X1; e = _X2; i = _X4; _YC = -da / (A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h);
        _ZC = m; u = _Y1; t = _Y2; C = _Y4; _ZC = da / (A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h);
        _ZD = m; i = _X3; C = _Y3; _ZD = -da / (A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h);
        _YD = m; u = _Z1; t = _Z2; C = _Z3; _YD = da / (A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h);
        _XD = m; z = _Y1; e = _Y2; i = _Y3; _XD = -da / (A * e * C - A * t * i - z * r * C + z * t * h + u * r * i - u * e * h);
        P1 = _X1 * _X1 + _Y1 * _Y1 + _Z1 * _Z1; P2 = _X2 * _X2 + _Y2 * _Y2 + _Z2 * _Z2; P3 = _X3 * _X3 + _Y3 * _Y3 + _Z3 * _Z3;
        P4 = _X4 * _X4 + _Y4 * _Y4 + _Z4 * _Z4; _XE = P1 / _XA + P2 / _XB + P3 / _XC + P4 / _XD;
        _YE = P1 / _YA + P2 / _YB + P3 / _YC + P4 / _YD; _ZE = P1 / _ZA + P2 / _ZB + P3 / _ZC + P4 / _ZD; _rst = 0;
    }

    public void processX(double _R1, double _R2, double _R3, double _R4) {
        double y;

        y = 999999;
        _X = Math.Pow((y - _R1), 2) / _XA + Math.Pow((y - _R2), 2) / _XB + Math.Pow((y - _R3), 2) / _XC + Math.Pow((y - _R4), 2) / _XD - _XE;// goto2
                                                                                                                                             //goto2
    }

    public void processY(double _R1, double _R2, double _R3, double _R4) {
        double y;

        y = 999999;
        _Y = Math.Pow(y - _R1, 2) / _YA + Math.Pow(y - _R2, 2) / _YB + Math.Pow(y - _R3, 2) / _YC + Math.Pow(y - _R4, 2) / _YD - _YE; //goto2
                                                                                                                                      //goto2
    }

    public void processZ(double _R1, double _R2, double _R3, double _R4) {
        double y;

        y = 999999;
        _Z = Math.Pow(y - _R1, 2) / _ZA + Math.Pow(y - _R2, 2) / _ZB + Math.Pow(y - _R3, 2) / _ZC + Math.Pow(y - _R4, 2) / _ZD - _ZE; //goto2
                                                                                                                                      //goto2
    }
}
