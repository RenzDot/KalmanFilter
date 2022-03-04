using System;
using RenzLibraries;
using System.Collections.Generic;
using System.Linq;

namespace KalmanFilter
{
    /*
    To Do:
    - Compare Unscented vs Standard Kalman on sin wave
    */
    public class Program
    {
        // Uncomment the following line to resolve.
        static void Main() {}
    }

    public interface IMeasurement { }

    public interface ITransistion { }

    public class MeasurementSpace: IMeasurement {
        public MeasurementSpace() {
            
        }
    }

    public class StateTransitionModel: ITransistion {
        Matrix stateTransition;
        public StateTransitionModel(Matrix StateTransition) {
            stateTransition = StateTransition;
        }
        public List<double> Next(List<double> state) {
            if (stateTransition.GetRow(0).Length != state.Count) { throw new ArgumentException($"Expected number of states to be {stateTransition.GetRow(0).Length}, but is instead {state.Count}"); }
            Matrix X = new Matrix(1, state.Count);
            X.SetRow(0, state.ToArray());
            X = X.Transpose();
            return stateTransition.Multiply(X).Transpose().GetRow(0).ToList();
        }
    }

    public interface ISigmaPointGenerator {
        Matrix GetSigmaPoints(List<double> mean, Matrix covariance);
        List<double> GetMeanWeight();
        List<double> GetCovarianceWeight();
    }

    public class MerweScaledSigmaPoints : ISigmaPointGenerator
    {
        int n = 2;
        double a = 0.3;// 0 to 1
        double b = 2.0;// 
        double k = 0;// (3 - n)
        double lambda;
        static readonly double million = 1000000;

        public MerweScaledSigmaPoints() { }
        public MerweScaledSigmaPoints(int dimension, double alpha, double beta, double kappa) {
            n = dimension;  // Dimensionality of the state. 2n+1 weights will be generated
            a = alpha;      // Determines spread of sigma points around mean
            b = beta;       // Incorporation of prior mean distribution knowledge
            k = kappa;      // Secondary scaling factor
            lambda = a * a * (n + k) - n;
        }
        //Tests:
        // 1x1 Matrcies produces 1x1 Matrix outputs
        // n=1 => sigmas.length == 5
        // All sigma points are within 1 standard deviation

        public Matrix GetSigmaPoints(List<double> mean, Matrix covariance) {
            if (mean.Count != n) {//Check mean has the correct dimension of n
                throw new ArgumentOutOfRangeException("mean", $"expected mean to have {n} items, but it has {mean.Count}");
            };

            if (!covariance.IsSymmetric()) {
                throw new ArgumentException("Covariance is not symmetric");
            }

            //Create Identity if P is 1x1
            Matrix P = covariance;
            if (covariance.GetRow(0).Length == 1 && covariance.GetColumn(0).Length == 1) {
                P = CreateScaledIdentity(covariance.Get(0,0));
            }

            int sigmaCount = 2 * n + 1;
            var sigmas = new Matrix(sigmaCount, n);
            double lambda = (a * a) * (n + k) - n;
            Matrix scaledP = P.Multiply(lambda * million + n * million).Multiply(1/million);
            Matrix U = scaledP.Cholesky(true);

            for (int i = 0; i < sigmaCount; i++) {
                sigmas.SetRow(i, mean.ToArray());
            }

            for (int j = 0; j < sigmaCount/2; j++) {
                var pointA = new List<double>(mean);
                for (int k = 0; k < pointA.Count; k++) {
                    pointA[k] = pointA[k] + U.GetRow(j)[k];
                }
                var pointB = new List<double>(mean);
                for (int k = 0; k < pointB.Count; k++) {
                    pointB[k] = pointB[k] - U.GetRow(j)[k];
                }
                sigmas.SetRow(j + 1, pointA.ToArray());
                sigmas.SetRow(n + j + 1, pointB.ToArray());
            }
            return sigmas;
        }

        public List<double> GetMeanWeight() {
            var meanWeight = new List<double>();
            meanWeight.Add(lambda / (n + lambda));

            double weight = 1 / (2 * (n + lambda));
            for(int i = 0; i < (2 * n); i++ ) {
                meanWeight.Add(weight);
            }
            return meanWeight;
        }

        public List<double> GetCovarianceWeight() {
            var covarianceWeight = new List<double>();
            covarianceWeight.Add(lambda / (n + lambda) + 1 - a * a + b);

            double weight = 1 / (2 * (n + lambda));
            for (int i = 0; i < (2 * n); i++) {
                covarianceWeight.Add(weight);
            }
            return covarianceWeight;
        }

        public Matrix CreateScaledIdentity(double scalar) {
            Matrix identity = new Matrix(n, n);
            for (int i = 0; i < n; i++) {
                identity.Set(i, i, scalar);
            }
            return identity;
        }
    }

    public class UnscentedKalman : IKalman
    {
        Matrix P, Q, X, Y, Pbar;
        List<double> meanWeight, covarianceWeight, xBar;
        MeasurementSpace measurementSpace;
        ISigmaPointGenerator sigmaGenerator;
        StateTransitionModel stateTransitionModel;

        public UnscentedKalman() { }

        public UnscentedKalman(List<double> mean, Matrix covariance, ISigmaPointGenerator sigmaPointGenerator, StateTransitionModel stateTransitionModel_F, MeasurementSpace measurementSpace_H, Matrix processNoiseCovariance) {
            this.sigmaGenerator = sigmaPointGenerator;
            this.P = covariance;
            this.stateTransitionModel = stateTransitionModel_F;
            this.measurementSpace = measurementSpace_H;
            Q = processNoiseCovariance;
            meanWeight = this.sigmaGenerator.GetMeanWeight();
            covarianceWeight = this.sigmaGenerator.GetCovarianceWeight();
        }

        public void Predict() { }

        public List<double> GetWeightedSigmas(List<double> weights, Matrix y) {
            var xBar = new List<double>(new double[y.GetRow(0).Length]);
            double[] yRow;
            for (int i = 0; i < y.GetColumn(0).Length; i++) {
                double weight = weights[i];
                var weightedSigmas = new List<double>();

                yRow = y.GetRow(i);
                foreach (double sigma in yRow) {
                    weightedSigmas.Add(weight * sigma);
                }
                xBar = xBar.Zip(weightedSigmas, (x1, x2) => x1 + x2).ToList();
            }
            return xBar;
        }

        public Matrix GetWeightedCovariance(Matrix transformedSigmas, List<double> weightedSigmas, List<double> covarianceWeight, Matrix processNoiseCovariance) {
            Matrix y = transformedSigmas;
            List<double> xBar = weightedSigmas;
            List<double> Wc = covarianceWeight;
            Matrix Q = processNoiseCovariance;

            for (int i = 0; i < y.GetColumn(0).Length; i++) {
                y.SetRow(i, y.GetRow(i).Zip(xBar, (x1, x2) => x1 - x2 ).ToArray());
            }

            Matrix WcMatrix = new Matrix(Wc.Count, Wc.Count);
            for (int i = 0; i < Wc.Count; i++) {
                WcMatrix.Set(i, i, Wc.ElementAt(i));
            }

            return y.Transpose().Multiply(WcMatrix.Multiply(y));
        }

        public Matrix TransitionSigmas(StateTransitionModel transitionModelFx, Matrix sigmaPointsX) {
            int rowSize = sigmaPointsX.GetColumn(0).Length;
            int colSize = sigmaPointsX.GetRow(0).Length;
            List<double> transformedSigmas;
            var y = new Matrix(rowSize, colSize);
            for (int i = 0; i < rowSize; i++) {
                transformedSigmas = transitionModelFx.Next(sigmaPointsX.GetRow(i).ToList());
                y.SetRow(i, transformedSigmas.ToArray());
            }
            return y;
        }

        public void Predict(List<double> mean, Matrix covariance, double timeStep) {
            P = covariance;
            X = sigmaGenerator.GetSigmaPoints(mean, P);

            // Y = f(x)
            Y = TransitionSigmas(stateTransitionModel, X);

            // xBar = Sum(wM * Y)
            xBar = GetWeightedSigmas(meanWeight, Y);

            // _P = Sum(    wC(Y - _x)*(Y - _x) + Q   )
            Pbar = GetWeightedCovariance(Y, xBar, covarianceWeight, Q);
        }

        public void Update(double[] Z) {
            // Z = h(Y)
            // uZ = Sum(wM * Z)
            // y = z - uZ
            // Pz = Sum(    wC*(Z - uZ)*Transpose(Z - uZ)  + R  )
            // K = ????
            // x = _x + K*y
            // P = P - K*Pz*Transpose(K)
        }

        public void UnscentedTransform() {

        }

        public double GetPos() {
            throw new NotImplementedException();
        }

        public double GetVel() {
            throw new NotImplementedException();
        }
    }

    public class KalmanSimple : IKalman
    {
        double x, v, a, z, t, Px, Pv, Pxv;
        double S, P, R, Y, Kx, Kv;

        public KalmanSimple(double init_x, double init_v, double time, double accel, double processErrorX, double processErrorV, double obvErrorX) {
            x = init_x;
            v = init_v;
            a = accel;
            t = time;
            Px = processErrorX * processErrorX;
            Pv = processErrorV * processErrorV;
            R = obvErrorX;
        }

        public void Predict() {
            x = x + v * t;
            Pxv = Pv * t + 0.5 * a * t * t * t;
            Px = Px + Pv * t * t + 0.25 * a * t * t * t * t;
            Pv = Pv + a * t * t;
        }

        public void Update(double[] Z) {
            S = Px + R;
            Y = Z[0] - x;
            Kx = Px / S;
            Kv = Pxv / S;
            x = Kx * Y + x;
            v = Kv * Y + v;
            Px = Px * (1 - Kx);
            Pv = -Kv * (Pxv) + Pv;
        }

        public double GetPos() {
            return x;
        }

        public double GetVel() {
            return v;
        }

        public double GetP() {
            return Px;
        }

        public double GetResidual() {
            return Y;
        }
    }

    public class KalmanBowtie : IKalman
    {//Reference: https://www.youtube.com/watch?v=Fuy73n6_bBc&list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT&index=27

        double a, Rx, Rv, Px, Pv, t, Zx, Zv;
        double x1, y1, z1 = 0d;
        double Kx, Kv, Xm, _v, yx, yv, _XX, _YY, _ZZ;

        public KalmanBowtie
            (double x_init, double v_init, double accel, double time, double processErrorX, double processErrorV, double obvErrorX, double obvErrorV) {
            Px = processErrorX * processErrorX;
            Pv = processErrorV * processErrorV;
            Rx = obvErrorX * obvErrorX;
            Rv = obvErrorV * obvErrorV;
            t = time;
            a = accel;
            Xm = x_init;
            _v = v_init;
        }

        public void Predict() {
            Xm = Xm + _v + 0.5 * a * t * t;
            _v = _v + a * t;
            Px = Px + Pv * (t * t);
        }

        public void Update(double[] Z) {
            Assign(Z);

            yx = ObserveX();
            yv = ObserveV();

            Kx = Px / (Px + Rx);
            Kv = Pv / (Pv + Rv);

            Xm = Xm + Kx * yx;
            _v = _v + Kv * yv;

            Px = Px * (1 - Kx);
            Pv = Pv * (1 - Kv);
        }

        public void Assign(double[] Z) {
            Zx = Z[0];
            Zv = Z[1];
        }

        public double ObserveX() {
            return Zx - Xm;
        }
        public double ObserveV() {
            return Zv - _v;
        }

        public double GetPos() {
            return Xm;
        }

        public double GetVel() {
            return _v;
        }
    }

    public class Kalman : IKalman
    {//Source: https://www.youtube.com/watch?v=m5Bw1m8jJuY
        Matrix x = new Matrix(2, 1);
        Matrix P = new Matrix(2, 2);
        double accel_variance;
        double z_variance;
        double dt;

        public Kalman(double initial_x, double initial_v, double time, double accel_variance, double observation_variance) {
            x.SetRow(0, new double[] { initial_x });
            x.SetRow(1, new double[] { initial_v });
            P.SetRow(0, new double[] { 1, 0 });
            P.SetRow(1, new double[] { 0, 1 });
            this.accel_variance = accel_variance;
            this.z_variance = observation_variance * observation_variance;
            this.dt = time;
        }

        public void Predict() {
            //x = F x
            //P = F P Ft + G Gt a
            Matrix F = new Matrix(2, 2);
            F.SetRow(0, new double[] { 1, dt });
            F.SetRow(1, new double[] { 0, 1 });
            Matrix new_x = F.Multiply(x);

            Matrix G = new Matrix(2, 1);
            G.SetRow(0, new double[] { 0.5 * dt * dt });
            G.SetRow(1, new double[] { dt });
            Matrix new_P =
                F.Multiply(P).Multiply(F.Transpose()).Add(G.Multiply(G.Transpose()).Multiply(accel_variance));
            P = new_P;
            x = new_x;
        }

        public void Update(double[] measurment) {
            Matrix z = new Matrix(1, 1);
            Matrix R = new Matrix(1, 1);
            Matrix H = new Matrix(1, 2);
            Matrix I = new Matrix(2, 2);

            z.Set(0, 0, measurment[0]);
            R.Set(0, 0, z_variance);
            H.SetRow(0, new double[] { 1, 0 });
            I.SetRow(0, new double[] { 1, 0 });
            I.SetRow(1, new double[] { 0, 1 });
            Matrix y = z.Subtract(H.Multiply(x));                           //Y = Z - H * X
            Matrix S = (H.Multiply(P).Multiply(H.Transpose())).Add(R);      //S = H * P * Ht + R
            Matrix K = P.Multiply(H.Transpose()).Multiply(S.Invert_2x2());  //K = P * Ht * S^-1

            Matrix new_x = K.Multiply(y).Add(x);                            //X = K * Y + X
            Matrix new_P = (I.Subtract(K.Multiply(H))).Multiply(P);         //P = I - K * H * P

            P = new_P;
            x = new_x;
        }

        public Matrix Cov() {
            return P;
        }

        public Matrix Mean() {
            return x;
        }

        public double GetPos() {
            return x.Get(0, 0);
        }

        public double GetVel() {
            return x.Get(1, 0);
        }
    }

    public interface IKalman
    {
        void Predict();
        void Update(double[] Z);
        double GetPos();
        double GetVel();
    }

    public class AlphaBetaFilter : IKalman
    {
        double t;
        double x, v, a, b, r;
        public AlphaBetaFilter(double init_x, double time, double alpha, double beta) {
            t = time;
            x = init_x;
            a = alpha;
            b = beta;
        }

        public void Predict() {
            x += v * t;

        }

        public void Update(double[] Z) {
            r = Z[0] - x;
            a = 1 / r;
            x += a * r;
            v += (b * r) / t;
        }

        public double GetPos() {
            return x;
        }

        public double GetVel() {
            return v;
        }
    }
}
