using System;
using System.Collections.Generic;
using System.Linq;
using static System.Math;

namespace Isan {
    public interface Iisan {

        void Update(double _x, double _y, double _z, double _w);
        void UpdateStaggered(double[] _x, double[] _y, double[] _z, double[] _w);
        void AddCoordinates(double _x, double _y, double _z);
        List<double> GetX();
        List<double> GetY();
        List<double> GetZ();
        List<double> GetVelX();
        List<double> GetVelY();
        List<double> GetVelZ();
        List<double> GetVelocities();

    }

    public class IsanKalmanV3 : Iisan
    {
        string sw, sx, sy, sz, _t, _at;
        double a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, S, t, u, w, x, y, z;
        double ac, at, ob, t2, t1, Px, Pb, Pa, Py, Pc, Pz, Wz, fa, Kz, vw, wv, S4, Y4, Kw;
        double vx, xv, th, Pw, Ka, S1, Kv, Y1, vy, Ky, S2, yv, Y2, vz, zv, Oz, S3, Y3;
        double _x, _y, _z, xK, yK, zK, _Vx, _Vy, _Vz, xx, yy, zz, Pxv, Kx, Pyv, Pzv;
        double ar, la, arr, lar, br, brr, cr, crr, dr, drr, Kb, Kc;
        double lb, lbr, lc, lcr, ld, ldr;
        List<List<double>> coordinates = new List<List<double>>() { new List<double>() { }, new List<double>() { }, new List<double>() { } };
        List<List<double>> velocities = new List<List<double>>() { new List<double>() { }, new List<double>() { }, new List<double>() { } };

        public IsanKalmanV3() {
            sz = "station_"; sw = sz + "hq_imperial_a"; sx = sz + "proving_grounds";
            sy = sz + "kingdom_outpost_b"; sz += "kingdom_outpost_b_2"; t = 999999; _at = sw;
            e = 207744.954; f = -1550111.534; g = 486779.292; h = -160699.392; i = -46169.175;
            j = -140483.070; k = 103669.797; l = 799664.698; m = -264673.522; n = -4718.935;
            o = -230400.908; p = 345153.593; q = 41051.448; r = -43636.344; s = -1606.533;
            ac = 0.3; ob = 50; t1 = 0.6;
            Px = 25.0; Pa = 12.5; _Vx = 0; 
            Py = 25.0; Pb = 12.5; _Vy = 0; 
            Pz = 25.0; Pc = 12.5; _Vz = 0; 
            
        }

        public void Update(double r1, double r2, double r3, double r4) {
            _at = sx; a = Pow(t - r1, 2); ar = (a - la) / 4; arr = (ar - lar) / 4; la = a; lar = ar;
            ar += arr; br += brr; cr += crr; dr += drr; a += ar; b += br; c += cr; d += dr;
            _x = a / e + b / f + c / g + d / h - i; _y = a / j + b / k + c / l + d / m - n; _z = a / o + b / p + c / q + d / r - s;

            x += _Vx * t1; Pxv = Pa * t1 + th; Px += Pa * t2 + fa; Pa += at; S = Px + ob; Y1 = _x - x;
            Kx = Px / S; Ka = Pxv / S; x += Kx * Y1; _x = x; _Vx += Ka * Y1; Px *= 1 - Kx; Pa += -Ka * Pxv;

            y += _Vy * t1; Pyv = Pb * t1 + th; Py += Pb * t2 + fa; Pb += at; S = Py + ob; Y1 = _y - y;
            Ky = Py / S; Kb = Pyv / S; y += Ky * Y1; _y = y; _Vy += Kb * Y1; Py *= 1 - Ky; Pb += -Kb * Pyv;

            z += _Vz * t1; Pzv = Pc * t1 + th; Pz += Pc * t2 + fa; Pc += at; S = Pz + ob; Y1 =_z - z;
            Kz = Pz / S; Kc = Pzv / S; z += Kz * Y1; _z = z; _Vz += Kc * Y1; Pz *= 1 - Kz; Pc += -Kc * Pzv;
            AddCoordinates(_x, _y, _z);
            AddVelocities(_Vx, _Vy, _Vz);

            _at = sy; b = Pow(t - r2, 2); br = (b - lb) / 4; brr = (br - lbr) / 4; lb = b; lbr = br;
            ar += arr; br += brr; cr += crr; dr += drr; a += ar; b += br; c += cr; d += dr;
            _x = a / e + b / f + c / g + d / h - i; _y = a / j + b / k + c / l + d / m - n; _z = a / o + b / p + c / q + d / r - s;

            x += _Vx * t1; Pxv = Pa * t1 + th; Px += Pa * t2 + fa; Pa += at; S = Px + ob; Y1 = _x - x;
            Kx = Px / S; Ka = Pxv / S; x += Kx * Y1; _x = x; _Vx += Ka * Y1; Px *= 1 - Kx; Pa += -Ka * Pxv;

            y += _Vy * t1; Pyv = Pb * t1 + th; Py += Pb * t2 + fa; Pb += at; S = Py + ob; Y1 = _y - y;
            Ky = Py / S; Kb = Pyv / S; y += Ky * Y1; _y = y; _Vy += Kb * Y1; Py *= 1 - Ky; Pb += -Kb * Pyv;

            z += _Vz * t1; Pzv = Pc * t1 + th; Pz += Pc * t2 + fa; Pc += at; S = Pz + ob; Y1 = _z - z;
            Kz = Pz / S; Kc = Pzv / S; z += Kz * Y1; _z = z; _Vz += Kc * Y1; Pz *= 1 - Kz; Pc += -Kc * Pzv;
            AddCoordinates(_x, _y, _z);
            AddVelocities(_Vx, _Vy, _Vz);

            _at = sz; c = Pow(t - r3, 2); cr = (c - lc) / 4; crr = (cr - lcr) / 4; lc = c; lcr = cr;
            ar += arr; br += brr; cr += crr; dr += drr; a += ar; b += br; c += cr; d += dr;
            _x = a / e + b / f + c / g + d / h - i; _y = a / j + b / k + c / l + d / m - n; _z = a / o + b / p + c / q + d / r - s;

            x += _Vx * t1; Pxv = Pa * t1 + th; Px += Pa * t2 + fa; Pa += at; S = Px + ob; Y1 = _x - x;
            Kx = Px / S; Ka = Pxv / S; x += Kx * Y1; _x = x; _Vx += Ka * Y1; Px *= 1 - Kx; Pa += -Ka * Pxv;

            y += _Vy * t1; Pyv = Pb * t1 + th; Py += Pb * t2 + fa; Pb += at; S = Py + ob; Y1 = _y - y;
            Ky = Py / S; Kb = Pyv / S; y += Ky * Y1; _y = y; _Vy += Kb * Y1; Py *= 1 - Ky; Pb += -Kb * Pyv;

            z += _Vz * t1; Pzv = Pc * t1 + th; Pz += Pc * t2 + fa; Pc += at; S = Pz + ob; Y1 = _z - z;
            Kz = Pz / S; Kc = Pzv / S; z += Kz * Y1; _z = z; _Vz += Kc * Y1; Pz *= 1 - Kz; Pc += -Kc * Pzv;
            AddCoordinates(_x, _y, _z);
            AddVelocities(_Vx, _Vy, _Vz);
            
            _at = sw; d = Pow(t - r4, 2); dr = (d - ld) / 4; drr = (dr - ldr) / 4; ld = d; ldr = dr;
            ar += arr; br += brr; cr += crr; dr += drr; a += ar; b += br; c += cr; d += dr;
            _x = a / e + b / f + c / g + d / h - i; _y = a / j + b / k + c / l + d / m - n; _z = a / o + b / p + c / q + d / r - s;

            x += _Vx * t1; Pxv = Pa * t1 + th; Px += Pa * t2 + fa; Pa += at; S = Px + ob; Y1 = _x - x;
            Kx = Px / S; Ka = Pxv / S; x += Kx * Y1; _x = x; _Vx += Ka * Y1; Px *= 1 - Kx; Pa += -Ka * Pxv;

            y += _Vy * t1; Pyv = Pb * t1 + th; Py += Pb * t2 + fa; Pb += at; S = Py + ob; Y1 = _y - y;
            Ky = Py / S; Kb = Pyv / S; y += Ky * Y1; _y = y; _Vy += Kb * Y1; Py *= 1 - Ky; Pb += -Kb * Pyv;

            z += _Vz * t1; Pzv = Pc * t1 + th; Pz += Pc * t2 + fa; Pc += at; S = Pz + ob; Y1 = _z - z;
            Kz = Pz / S; Kc = Pzv / S; z += Kz * Y1; _z = z; _Vz += Kc * Y1; Pz *= 1 - Kz; Pc += -Kc * Pzv;
            AddCoordinates(_x, _y, _z);
            AddVelocities(_Vx, _Vy, _Vz);
        }

        public void UpdateStaggered(double[] x, double[] y, double[] z, double[] w) {
            int iPos = 0;
            int delay = 3;
            while (iPos + (delay * 4) < x.Length) {
                Update(x[iPos += delay], y[iPos += delay], z[iPos += delay], w[iPos += delay]);
            }
        }


        public void AddCoordinates(double xx, double yy, double zz) {
            coordinates[0].Add(xx);
            coordinates[1].Add(yy);
            coordinates[2].Add(zz);
        }

        public void AddVelocities(double vx, double vy, double vz) {
            velocities[0].Add(vx);
            velocities[1].Add(vy);
            velocities[2].Add(vz);
        }

        public List<double> GetX() {
            return coordinates[0];
        }

        public List<double> GetY() {
            return coordinates[1];
        }

        public List<double> GetZ() {
            return coordinates[2];
        }

        public List<double> GetVelX() {
            return velocities[0];
        }

        public List<double> GetVelY() {
            return velocities[1];
        }

        public List<double> GetVelZ() {
            return velocities[2];
        }

        public List<double> GetVelocities() {
            var vel = new List<double>() { };
            var x = velocities[0];
            var y = velocities[1];
            var z = velocities[2];
            for (int i = 0; i < velocities[0].Count; i++) {
                vel.Add(
                    Sqrt(Pow(x[i], 2) + Pow(y[i], 2) + Pow(z[i], 2))
                );
            }
            return vel;
        }
    }

    public class IsanKalmanV2 : Iisan
    {
        string sw, sx, sy, sz, _t;
        double a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, w, x, y, z;
        double ac, at, ob, t2, t1, Px, Pv, Qy, Py, Qz, Pz, Wz, fa, Kz, vw, wv, S4, Y4, Kw;
        double vx, xv, th, Pw, Ka, S1, Kv, Y1, vy, Ky, S2, yv, Y2, vz, zv, Oz, S3, Y3;
        double _x, _y, _z, xK, yK, zK;
        List<List<double>> coordinates = new List<List<double>>() { new List<double>() { }, new List<double>() { }, new List<double>() { } };
        List<List<double>> velocities = new List<List<double>>() { new List<double>() { }, new List<double>() { }, new List<double>() { } };

        public IsanKalmanV2() {
            sz = "station_"; sw = sz + "hq_imperial_a"; sx = sz + "proving_grounds";
            sy = sz + "kingdom_outpost_b"; sz += "kingdom_outpost_b_2"; t = 999999; _t = sw;
            e = 207744.954; f = -1550111.534; g = 486779.292; h = -160699.392; i = -46169.175;
            j = -140483.070; k = 103669.797; l = 799664.698; m = -264673.522; n = -4718.935;
            o = -230400.908; p = 345153.593; q = 41051.448; r = -43636.344; s = -1606.533;
            ac = 0.3; ob = 50; Px = 25; Pv = 12; t1 = 1.45; t2 = Pow(t1, 2); th = 0.5 * ac * Pow(t1, 3); at = ac * t2;
            Qy = Px; Qz = Px; Oz = Px; Py = Pv; Pz = Pv; fa = 0.25 * ac * Pow(t1, 4);
            Kv = 0.1; Ky = 0.1; Kz = 0.1; xK = 0.3; yK = 0.3; zK = 0.3;
        }

        public void Update(double r1, double r2, double r3, double r4) {
            _t = sx; a = Pow(t - r1, 2); x =_x + vx * t1; y =_y + vy * t1; z = _z + vz * t1;
            xv = Pv * t1 + th; Pv += at; S1 = Px + ob; xK = Px / S1; Kv = xv / S1; Pv += -Kv * xv;
            Y1 = a / e + b / f + c / g + d / h - i - x; Y2 = a / j + b / k + c / l + d / m - n - y; Y3 = a / o + b / p + c / q + d / r - s - z;
            vx += Kv * Y1; vy += Ky * Y2; vz += Kz * Y3; _x = x + xK * Y1; _y = y + yK * Y2; _z = z + zK * Y3;
            AddCoordinates(_x, _y, _z);
            AddVelocities(vx, vy, vz);

            _t = sy; b = Pow(t - r2, 2); x = _x + vx * t1; y = _y + vy * t1; z = _z + vz * t1;
            Y1 = a / e + b / f + c / g + d / h - i - x; Y2 = a / j + b / k + c / l + d / m - n - y; Y3 = a / o + b / p + c / q + d / r - s - z;
            yv = Py * t1 + th; Py += at; S2 = Qy + ob; yK = Qy / S2; Ky = yv / S2; Py += -Ky * yv;
            vx += Kv * Y1; vy += Ky * Y2; vz += Kz * Y3; _x = x + xK * Y1; _y = y + yK * Y2; _z = z + zK * Y3;
            AddCoordinates(_x, _y, _z);
            AddVelocities(vx, vy, vz);

            _t = sz; c = Pow(t - r3, 2); x = _x + vx * t1; y = _y + vy * t1; z = _z + vz * t1;
            Y1 = a / e + b / f + c / g + d / h - i - x; Y2 = a / j + b / k + c / l + d / m - n - y; Y3 = a / o + b / p + c / q + d / r - s - z;
            zv = Pz * t1 + th; Pz += at; S3 = Oz + ob; zK = Oz / S3; Kz = zv / S3; Pz += -Kz * zv;
            vx += Kv * Y1; vy += Ky * Y2; vz += Kz * Y3; _x = x + xK * Y1; _y = y + yK * Y2; _z = z + zK * Y3;
            AddCoordinates(_x, _y, _z);
            AddVelocities(vx, vy, vz);

            _t = sw; d = Pow(t - r4, 2); x = _x + vx * t1; y = _y + vy * t1; z = _z + vz * t1;
            Y1 = a / e + b / f + c / g + d / h - i - x; Y2 = a / j + b / k + c / l + d / m - n - y; Y3 = a / o + b / p + c / q + d / r - s - z;
            vx += Kv * Y1; vy += Ky * Y2; vz += Kz * Y3; _x = x + xK * Y1; _y = y + yK * Y2; _z = z + zK * Y3;
            AddCoordinates(_x, _y, _z);
            AddVelocities(vx, vy, vz);
        }

        public void UpdateStaggered(double[] x, double[] y, double[] z, double[] w) {
            int iPos = 0;
            int delay = 4;
            while (iPos + (delay*4) < x.Length) {
                Update(x[iPos += delay], y[iPos += delay], z[iPos += delay], w[iPos += delay]);
            }
        }


        public void AddCoordinates(double xx, double yy, double zz) {
            coordinates[0].Add(xx);
            coordinates[1].Add(yy);
            coordinates[2].Add(zz);
        }

        public void AddVelocities(double vx, double vy, double vz) {
            velocities[0].Add(vx);
            velocities[1].Add(vy);
            velocities[2].Add(vz);
        }

        public List<double> GetX() {
            return coordinates[0];
        }

        public List<double> GetY() {
            return coordinates[1];
        }

        public List<double> GetZ() {
            return coordinates[2];
        }

        public List<double> GetVelX() {
            return velocities[0];
        }

        public List<double> GetVelY() {
            return velocities[1];
        }

        public List<double> GetVelZ() {
            return velocities[2];
        }

        public List<double> GetVelocities() {
            var vel = new List<double>() { };
            var x = velocities[0];
            var y = velocities[1];
            var z = velocities[2];
            for (int i = 0; i < velocities[0].Count; i++) {
                vel.Add(
                    Sqrt(Pow(x[i], 2) + Pow(y[i], 2) + Pow(z[i], 2))
                );
            }
            return vel;
        }
    }


    public class IsanKalman : Iisan
    {
        string sw, sx, sy, sz, _t;
        double a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, w, x, y, z;
        double ac, at, ob, t2, t1, Px, Pv, Qy, Py, Qz, Pz, Wz, fa, Kz, vw, wv, S4, Y4, Kw;
        double vx, xv, th, Pw, Ka, S1, Kv, Y1, vy, Ky, S2, yv, Y2, vz, zv, Oz, S3, Y3;
        double _x, _y, _z, _a;
        List<List<double>> coordinates = new List<List<double>>() { new List<double>() { }, new List<double>() { }, new List<double>() { } };
        List<List<double>> velocities = new List<List<double>>() { new List<double>() { }, new List<double>() { }, new List<double>() { } };

        public IsanKalman() {
            sz = "station_"; sw = sz + "hq_imperial_a"; sx = sz + "proving_grounds";
            sy = sz + "kingdom_outpost_b"; sz += "kingdom_outpost_b_2"; t = 999999; _t = sw;
            e = 207744.954; f = -1550111.534; g = 486779.292; h = -160699.392; i = -46169.175;
            j = -140483.070; k = 103669.797; l = 799664.698; m = -264673.522; n = -4718.935;
            o = -230400.908; p = 345153.593; q = 41051.448; r = -43636.344; s = -1606.533;
            ac = 0.3; ob = 50; t1 = 1.45; t2 = Pow(t1, 2); th = 0.5 * ac * Pow(t1,3); at = ac * t2;
            Px = 25; Pv = 12; Qy = Px; Py = Pv; Qz = Px; Pz = Pv; Wz = Px; Pw = Pv; fa = 0.25 * ac * Pow(t1, 4);
        }

        //To DO: Possibly only update P every few ticks
        public void Update(double r1, double r2, double r3, double r4) {
            _t = sx; x += vx * t1; xv = Pv * t1 + th; Px += Pv * t2 + fa; Pv += at; S1 = Px + ob; Y1 = r1 - x;
            Ka = Px / S1; Kv = xv / S1; x += Ka * Y1; vx += Kv * Y1; Px *= 1 - Ka; Pv += -Kv * xv; a = Pow(t - x, 2);
            _x = a / e + b / f + c / g + d / h - i; _y = a / j + b / k + c / l + d / m - n; _z = a / o + b / p + c / q + d / r - s;
            AddCoordinates(_x, _y, _z);

            _t = sy; y += vy * t1; yv = Py * t1 + th; Qy += Py * t2 + fa; Py += at; S2 = Qy + ob; Y2 = r2 - y;
            Ka = Qy / S2; Ky = yv / S2; y += Ka * Y2; vy += Ky * Y2; Qy *= 1 - Ka; Py += -Ky * yv; b = Pow(t - y, 2);
            _x = a / e + b / f + c / g + d / h - i; _y = a / j + b / k + c / l + d / m - n; _z = a / o + b / p + c / q + d / r - s;
            AddCoordinates(_x, _y, _z);

            _t = sz; z += vz * t1; zv = Pz * t1 + th; Oz += Pz * t2 + fa; Pz += at; S3 = Oz + ob; Y3 = r3 - z;
            Ka = Oz / S3; Kz = zv / S3; z += Ka * Y3; vz += Kz * Y3; Oz *= 1 - Ka; Pz += -Kz * zv; c = Pow(t - z, 2);
            _x = a / e + b / f + c / g + d / h - i; _y = a / j + b / k + c / l + d / m - n; _z = a / o + b / p + c / q + d / r - s;
            AddCoordinates(_x, _y, _z);

            _t = sw; w += vw * t1; wv = Pw * t1 + th; Wz += Pw * t2 + fa; Pw += at; S4 = Wz + ob; Y4 = r4 - w;
            Ka = Wz / S4; Kw = wv / S4; w += Ka * Y4; vw += Kw * Y4; Wz *= 1 - Ka; Pw += -Kw * wv; d = Pow(t - w, 2);
            _x = a / e + b / f + c / g + d / h - i; _y = a / j + b / k + c / l + d / m - n; _z = a / o + b / p + c / q + d / r - s;
            AddCoordinates(_x, _y, _z);

        }

        public void UpdateStaggered(double[] x, double[] y, double[] z, double[] w) {
            int iPos = 0;
            while (iPos + 12 < x.Length) {
                Update(x[iPos += 3], y[iPos += 3], z[iPos += 3], w[iPos += 3]);
            }
        }


        public void AddCoordinates(double xx, double yy, double zz) {
            coordinates[0].Add(xx);
            coordinates[1].Add(yy);
            coordinates[2].Add(zz);
        }

        //Wrong, it misses out w
        public void AddVelocities(double vx, double vy, double vz) {
            velocities[0].Add(vx);
            velocities[1].Add(vy);
            velocities[2].Add(vz);
        }

        public List<double> GetX() {
            return coordinates[0];
        }

        public List<double> GetY() {
            return coordinates[1];
        }

        public List<double> GetZ() {
            return coordinates[2];
        }

        public List<double> GetVelX() {
            return velocities[0];
        }

        public List<double> GetVelY() {
            return velocities[1];
        }

        public List<double> GetVelZ() {
            return velocities[2];
        }

        public List<double> GetVelocities() {
            var vel = new List<double>() { };
            var x = velocities[0];
            var y = velocities[1];
            var z = velocities[2];
            for (int i = 0; i < velocities[0].Count; i++) {
                vel.Add(
                    Sqrt(   Pow(x[i], 2) + Pow(y[i], 2) + Pow(z[i], 2)  )    
                );
            }
            return vel;
        }
    }

    public class Isan2Mono : Iisan
    {
        public double _x, _y, _z, _w;
        public double[] _ax, _ay, _az, _aw;
        string x, y, z, w, _t;
        double a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u;
        double ar, arr, la, lar, br, brr, cr, crr, dr, drr, lb, lbr, lc, lcr, ldr, ld;
        List<List<double>> coordinates;
        //Unused: v

        public Isan2Mono() {
            Reset();
            z = "station_"; w = z + "hq_imperial_a"; x = z + "proving_grounds";
            y = z + "kingdom_outpost_b"; z += "kingdom_outpost_b_2";
            t = 999999; _t = w;
            e = 207744.954; f = -1550111.534; g = 486779.292; h = -160699.392; i = -46169.175;
            j = -140483.070; k = 103669.797; l = 799664.698; m = -264673.522; n = -4718.935;
            o = -230400.908; p = 345153.593; q = 41051.448; r = -43636.344; s = -1606.533;
        }

        public void Update(double _ax, double _ay, double _az, double _aw) {
            for (int ii = 0; ii < 3; ii++) {
                switch (u) {
                    case 6: _t = x; a = Pow(t - _ax, 2); ar = (a - la) / 4; arr = (ar - lar) / 4; la = a; lar = ar; u = 7; break;
                    case 7: _t = y; b = Pow(t - _ay, 2); br = (b - lb) / 4; brr = (br - lbr) / 4; lb = b; lbr = br; u = 8; break;
                    case 8: _t = z; c = Pow(t - _az, 2); cr = (c - lc) / 4; crr = (cr - lcr) / 4; lc = c; lcr = cr; u = 9; break;
                    case 9: _t = w; d = Pow(t - _aw, 2); dr = (d - ld) / 4; drr = (dr - ldr) / 4; ld = d; ldr = dr; u = 6; break;
                }
                
                ar += arr; br += brr; cr += crr; dr += drr; a += ar; b += br; c += cr; d += dr;
                _x = a / e + b / f + c / g + d / h - i; _y = a / j + b / k + c / l + d / m - n; _z = a / o + b / p + c / q + d / r - s;
                AddCoordinates(_x, _y, _z);
            }
        }

        public void UpdateStaggered(double[] x, double[] y, double[] z, double[] w) {
            int iPos = 0;
            while (iPos + 12 < x.Length) {
                Update( x[iPos+=3], y[iPos+=3], z[iPos+=3], w[iPos+=3] );
            }
        }

        public void AddCoordinates(double xx, double yy, double zz) {
            coordinates[0].Add(xx);
            coordinates[1].Add(yy);
            coordinates[2].Add(zz);
        }

        public void Reset() {
            u = 6;
            coordinates = new List<List<double>>() { new List<double>() { }, new List<double>() { }, new List<double>() { } };
        }
        
        public List<double> GetX() {
            return coordinates[0];
        }

        public List<double> GetY() {
            return coordinates[1];
        }

        public List<double> GetZ() {
            return coordinates[2];
        }

        public void AddVelocities(double _vx, double _vy, double _vw) {
            throw new NotImplementedException();
        }

        public List<double> GetVelX() {
            return new List<double>();
        }

        public List<double> GetVelY() {
            return new List<double>();
        }

        public List<double> GetVelZ() {
            return new List<double>();
        }

        public List<double> GetVelocities() {
            var time = 0.6d;
            var vel = new List<double>() { };
            for (int i = 0; i < (coordinates[0].Count - 1); i++) {
                double velX = (coordinates[0][i+1] - coordinates[0][i]);
                double velY = (coordinates[1][i+1] - coordinates[1][i]);
                double velZ = (coordinates[2][i+1] - coordinates[2][i]);
                vel.Add(Sqrt((velX * velX) + (velY * velY) + (velZ * velZ)) / time);
            }
            return vel;
        }
    }

}

