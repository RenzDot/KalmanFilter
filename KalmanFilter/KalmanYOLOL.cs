using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static System.Math;

namespace KalmanFilter
{

    public class KalmanYOLOL : IKalman
    {
        double x, v, a, z, t, Px, Pv, Pxv;
        double S, R, Y, Kx, Kv;
        double t1, t2, t3, t4, th, at, fa, h, f;

        public KalmanYOLOL(double init_x, double init_v, double time, double accel, double processErrorX, double processErrorV, double obvErrorX) {
            x = init_x;
            v = init_v;
            a = accel;
            t = time;
            Px = processErrorX * processErrorX;
            Pv = processErrorV * processErrorV;
            R = obvErrorX;

            h = 0.5; f = 0.25;
            t1 = t; t2 = Pow(t, 2); t3 = Pow(t, 3); t4 = Pow(t, 4);
            th = h * a * t3;
            at = a * t2;
            fa = f * a * t4;
        }

        public void Predict() {
            x += v * t;
            Pxv = Pv * t + th;
            Px += Pv * t2 + fa;
            Pv += at;
            S = Px + R;
        }

        public void Update(double[] z) {
            Y = z[0] - x;
            Kx = Px / S;
            Kv = Pxv / S;
            x += Kx * Y;
            v += Kv * Y;
            Px *= 1 - Kx;
            Pv += -Kv * Pxv;
        }

        public double GetPos() {
            return x;
        }

        public double GetVel() {
            return v;
        }
    }

    public class KalmanYOLOL_Old
    {
        double _X1, _X2;
        double xX1, xX2;
        double P1, P2, P3, P4;
        double pP1, pP2, pP3, pP4;
        double a;

        double F1, F2, F3, F4;
        double _F1, _F2, _F3, _F4;
        double Ft1, Ft2, Ft3, Ft4;
        double dt = 0.2;

        double G1, G2;
        double Gt1, Gt2;
        double Q1, Q2, Q3, Q4;
        public KalmanYOLOL_Old(double initial_x, double initial_v, double a_variance) {
            _X1 = initial_x;
            _X2 = initial_v;

            P1 = 1; P2 = 0;
            P3 = 0; P4 = 1;

            a = a_variance;
        }

        public void Predict(double dt) {
            F1 = 1; F2 = dt;
            F3 = 0; F4 = 1;

            Ft1 = 1; Ft2 = 0;
            Ft3 = dt; Ft4 = 1;

            a = 0.1;

            //X = F*X
            xX1 = (F1 * _X1) + (F2 * _X2);
            xX2 = (F3 * _X1) + (F4 * _X2);
            _X1 = xX1;
            _X2 = xX2;

            G1 = 0.5 * dt * dt;
            G2 = dt;

            Gt1 = G1; Gt2 = dt;

            // P = F*P*Ft + a(G*Gt)
            // _P = F * P
            pP1 = (F1 * P1) + (F2 * P3); pP2 = (F1 * P2) + (F2 * P4);
            pP3 = (F3 * P1) + (F4 * P3); pP4 = (F3 * P2) + (F4 * P4);

            // P = _P * Ft
            P1 = (pP1 * Ft1) + (pP2 * Ft3); P2 = (pP1 * Ft2) + (pP2 * Ft4);
            P3 = (pP3 * Ft1) + (pP4 * Ft3); P4 = (pP3 * Ft2) + (pP4 * Ft4);

            //Q = a(G*Gt)
            Q1 = a * (G1 * Gt1); Q2 = a * (G1 * Gt2);
            Q3 = a * (G2 * Gt1); Q4 = a * (G2 * Gt2);

            //P = P + Q
            P1 += Q1; P2 += Q2;//P1=0.01  P2=0.002
            P3 += Q3; P4 += Q4;//P3=0.003 P4=0.966
        }

        double _Z;
        double R1;
        double H1, H2;
        double Ht1, Ht2;
        double I1, I2, I3, I4;
        double T1, T2, T3, T4;
        double Y1;
        double S1, S2;
        double K1, K2;
        public void Update(double measurement, double variance) {
            _Z = measurement;
            R1 = variance;

            H1 = 1; H2 = 0;

            Ht1 = 1;
            Ht2 = 0;

            I1 = 1; I2 = 0;
            I3 = 0; I4 = 1;

            //Y = Z - H*X
            Y1 = _Z - ((H1 * _X1) + (H2 * _X2));

            //S = H*P*Ht + R
            //S = H*P
            S1 = (H1 * P1) + (H2 * P3); S2 = (H1 * P2) + (H2 * P4);

            //S = (S * Ht) + R
            S1 = ((S1 * Ht1) + (S2 * Ht2)) + R1;

            //S Inverse = 1/S
            //K = P * Ht * S Inverse
            K1 = ((P1 * Ht1) + (P2 * Ht2)) * (1/S1);
            K2 = ((P3 * Ht1) + (P4 * Ht2)) * (1/S1);

            //X = K*Y + X
            _X1 = K1 * Y1 + _X1;
            _X2 = K2 * Y1 + _X2;

            //P = (I - (K*H))P
            //I - (K*H)
            T1 = I1 - (K1 * H1); T2 = I2 - (K1 * H2);
            T3 = I3 - (K2 * H1); T4 = I4 - (K2 * H2);

            //P = (_P)P
            pP1 = (T1 * P1) + (T2 * P3); pP2 = (T1 * P2) + (T2 * P4);
            pP3 = (T3 * P1) + (T4 * P3); pP4 = (T3 * P2) + (T4 * P4);

            P1 = pP1; P2 = pP2;
            P3 = pP3; P4 = pP4;
        }

        public double[][] Cov() {
            double[][] P = new  double[2][];
            P[0] = new double[] { P1, P2 };
            P[1] = new double[] { P3, P4 };
            return P;
        }

        public double[][] Mean() {
            double[][] x = new double[2][];
            x[0] = new double[] { _X1 };
            x[1] = new double[] { _X2 };
            return x;
        }

        public void Step_1D(double _Z) {
            R1 = 0.01;
            dt = 0.2;

            F1 = 1; F2 = dt;
            F3 = 0; F4 = 1;

            P1 = 1; P2 = 0;
            P3 = 0; P4 = 1;

            Ft1 = 1; Ft2 = 0;
            Ft3 = dt; Ft4 = 1;

            H1 = 1; H2 = 0;

            Ht1 = 1;
            Ht2 = 0;

            I1 = 1; I2 = 0;
            I3 = 0; I4 = 1;

            xX1 = (F1 * _X1) + (F2 * _X2);
            xX2 = (F3 * _X1) + (F4 * _X2);
            _X1 = xX1;
            _X2 = xX2;

            G1 = 0.5 * dt * dt;
            G2 = dt;

            Gt1 = G1; Gt2 = dt;

            pP1 = (F1 * P1) + (F2 * P3); pP2 = (F1 * P2) + (F2 * P4);
            pP3 = (F3 * P1) + (F4 * P3); pP4 = (F3 * P2) + (F4 * P4);

            P1 = (pP1 * Ft1) + (pP2 * Ft3); P2 = (pP1 * Ft2) + (pP2 * Ft4);
            P3 = (pP3 * Ft1) + (pP4 * Ft3); P4 = (pP3 * Ft2) + (pP4 * Ft4);

            Q1 = a * (G1 * Gt1); Q2 = a * (G1 * Gt2);
            Q3 = a * (G2 * Gt1); Q4 = a * (G2 * Gt2);

            P1 += Q1; P2 += Q2;
            P3 += Q3; P4 += Q4;

            Y1 = _Z - ((H1 * _X1) + (H2 * _X2));

            S1 = (H1 * P1) + (H2 * P3); S2 = (H1 * P2) + (H2 * P4);
            S1 = ((S1 * Ht1) + (S2 * Ht2)) + R1;

            K1 = ((P1 * Ht1) + (P2 * Ht2)) * (1 / S1);
            K2 = ((P3 * Ht1) + (P4 * Ht2)) * (1 / S1);

            _X1 = K1 * Y1 + _X1;
            _X2 = K2 * Y1 + _X2;

            T1 = I1 - (K1 * H1); T2 = I2 - (K1 * H2);
            T3 = I3 - (K2 * H1); T4 = I4 - (K2 * H2);

            pP1 = (T1 * P1) + (T2 * P3); pP2 = (T1 * P2) + (T2 * P4);
            pP3 = (T3 * P1) + (T4 * P3); pP4 = (T3 * P2) + (T4 * P4);

            P1 = pP1; P2 = pP2;
            P3 = pP3; P4 = pP4;
        }

       

    }
}
