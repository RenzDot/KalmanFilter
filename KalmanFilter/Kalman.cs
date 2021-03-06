using System;
using RenzLibraries;
using System.Collections.Generic;
using System.Linq;

namespace KalmanFilter
{
    /*
    To Do:
    - Compare Unscented vs Standard Kalman on sin wave
    - Figure out inversion for any size matrix, instead of just 2x2 inversion
    */
    public class Program {
        // Uncomment the following line to resolve.
        static void Main() { }
    }

    public interface ITransistion { }

    public class MeasurementSpace : ITransistion {
        Matrix measurementSpace;
        public MeasurementSpace() {

        }

        public Matrix Next(Matrix sigmas) {
            int rowSize = sigmas.GetColumn(0).Length;
            int colSize = (sigmas.GetRow(0).Length / 2);
            Matrix H = new Matrix(rowSize, colSize);
            for (int i = 0; i < rowSize; i++) {
                for (int j = 0; j < colSize; j++) {
                    H.Set(i, j, sigmas.Get(i, j * 2));
                }
            }
            return H;
        }
    }

    public class StateTransitionModel : ITransistion {
        Matrix stateTransition;
        public StateTransitionModel(Matrix StateTransition) {
            stateTransition = StateTransition;
        }
        public List<double> Next(List<double> state) {
            if (stateTransition.GetRow(0).Length != state.Count) { throw new ArgumentException($"Expected number of states to be {stateTransition.GetRow(0).Length}, but is instead {state.Count}"); }
            Matrix X = new Matrix(1, state.Count);
            X.SetRow(0, state.ToArray());
            return stateTransition.Multiply(X.Transpose()).Transpose().GetRow(0).ToList();
        }
    }

    /*
    public class DiscreteWhiteNoiseQ {
        Matrix Q;
        public DiscreteWhiteNoiseQ(int dimensions, double dt) {
            switch (dimensions) {
                case 2:
                    Q = new Matrix(2, 2);
                    Q.SetRow(0, new double[] { 0.25 * Math.Pow(dt, 4), 0.5 * Math.Pow(dt, 3) });
                    Q.SetRow(1, new double[] { 0.5 * Math.Pow(dt, 3), dt * dt });
                    break;
                case 3:
                    Q = new Matrix(3, 3);
                    Q.SetRow(0, new double[] { 0.25 * Math.Pow(dt, 4), 0.5 * Math.Pow(dt, 3), 0.5 * dt * dt });
                    Q.SetRow(1, new double[] { 0.5 * Math.Pow(dt, 3), dt * dt, dt });
                    Q.SetRow(2, new double[] { 0.5 * dt * dt, dt, 1 });
                    break;
                case 4:
                    Q = new Matrix(4, 4);
                    Q.SetRow(0, new double[] { Math.Pow(dt, 6) / 36, Math.Pow(dt, 5) / 12, Math.Pow(dt, 4) / 6, Math.Pow(dt, 3) / 6 });
                    Q.SetRow(1, new double[] { Math.Pow(dt, 5) / 12, Math.Pow(dt, 4) / 4, Math.Pow(dt, 3) / 2, Math.Pow(dt, 2) / 2 });
                    Q.SetRow(2, new double[] { Math.Pow(dt, 4) / 6, Math.Pow(dt, 3) / 2, Math.Pow(dt, 2), dt });
                    Q.SetRow(3, new double[] { Math.Pow(dt, 3) / 6, Math.Pow(dt, 2) / 2, dt, 1.0 });
                    break;
                default:
                    throw new ArgumentException("DiscreteWhiteNoiseQ must have a dimension value between 2 to 4");
            }
        }

        public Matrix GetQ() {
            return Q;
        }
    };*/

    public interface ISigmaPointGenerator {
        Matrix GetSigmaPoints(List<double> mean, Matrix covariance);
        List<double> GetMeanWeight();
        List<double> GetCovarianceWeight();
    }

    

    public class UnscentedKalman : IKalman {
        Matrix Q, R, X, Y, Zeta, K;
        Matrix P_Posterior, Pz, Pxz, P_Prior, residual_y;
        List<double> xPosterior, meanWeight, covarianceWeight, xPrior, Uz;
        MeasurementSpace measurementSpace;
        ISigmaPointGenerator sigmaGenerator;
        StateTransitionModel stateTransitionModel;

        public UnscentedKalman() { }

        public UnscentedKalman(
                List<double> initialMean, 
                Matrix initialCovariance,
                ISigmaPointGenerator sigmaPointGenerator,
                StateTransitionModel stateTransitionModel_F,
                MeasurementSpace measurementSpace_H,
                Matrix processNoiseCovariance_Q,
                Matrix Noise_R
            ) {
            this.sigmaGenerator = sigmaPointGenerator;
            meanWeight = this.sigmaGenerator.GetMeanWeight();
            covarianceWeight = this.sigmaGenerator.GetCovarianceWeight();
            this.P_Posterior = initialCovariance;
            this.stateTransitionModel = stateTransitionModel_F;
            this.measurementSpace = measurementSpace_H;
            Q = processNoiseCovariance_Q;
            R = Noise_R;

            xPosterior = CreateInitialX(initialMean);
        }

        public List<double> CreateInitialX(List<double> x) {
            double[] mean = new double[x.Count * 2];;
            for (int i = 0; i < x.Count; i++) {
                mean[i * 2] = x[i];
            }
            return mean.ToList();
        }

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

        public Matrix GetResidual(List<double> measurement, List<double> prior) {
            Matrix residual = new Matrix(1, prior.Count);
            residual.SetRow(0, measurement.Zip(prior, (z, u) => z - u).ToArray());
            return residual;
        }

        public Matrix GetCrossVariance(List<double> sigmasXbar, List<double> sigmasU, List<double> covWeight, Matrix sigmasY, Matrix sigmasZ) {
            int rowSize = sigmasY.GetRow(0).Length;
            int colSize = sigmasZ.GetRow(0).Length;
            Matrix Pxz = new Matrix(rowSize, colSize);
            Matrix outer = new Matrix(rowSize, colSize);
            List<double> dx, dz;
            double[] row;
            int n = sigmasY.GetColumn(0).Length;
            for (int i = 0; i < n; i++) {
                var rowX = sigmasY.GetRow(i);
                var rowZ = sigmasZ.GetRow(i);
                dx = sigmasY.GetRow(i).Zip(sigmasXbar, (yi, xi) => yi - xi).ToList();
                dz = sigmasZ.GetRow(i).Zip(sigmasU, (zi, ui) => zi - ui ).ToList();

                for (int j = 0; j < rowSize; j++) {
                    row = dz.Select(zi => dx[j] * zi).ToArray();
                    outer.SetRow(j, row);
                }
                outer = outer.Multiply(covWeight[i]);
                Pxz = Pxz.Add(outer);
                //Pxz += self.Wc[i] * outer(dx, dz)
            }
            return Pxz;
        }

        public Matrix GetWeightedCovariance(Matrix sigmas, List<double> weightedSigmas, List<double> covarianceWeight) {
            List<double> xBar = weightedSigmas;
            List<double> Wc = covarianceWeight;
            int rowSize = sigmas.GetColumn(0).Length;
            int colSize = sigmas.GetRow(0).Length;
            Matrix y = new Matrix(rowSize, colSize);

            for (int i = 0; i < rowSize; i++) {
                y.SetRow(i, sigmas.GetRow(i).Zip(xBar, (x1, x2) => x1 - x2 ).ToArray());
            }

            Matrix WcMatrix = new Matrix(Wc.Count, Wc.Count);
            for (int i = 0; i < Wc.Count; i++) {
                WcMatrix.Set(i, i, Wc.ElementAt(i));
            }

            return y.Transpose().Multiply(WcMatrix.Multiply(y));
        }

        public Matrix GetKalmanGain(Matrix crossVariance, Matrix covariance) {
            Matrix inverse = covariance.Invert_2x2();
            return crossVariance.Multiply(inverse);
        }

        public List<double> GetPosteriorX(List<double> x , Matrix kGain, Matrix residual) {
            if (residual.GetColumn(0).Length != 1) { 
                throw new ArgumentException($"Cannot obtain x posterior as residual matrix has {residual.GetColumn(0).Length} rows, when it should be one"); 
            }
            int rowSize = kGain.GetColumn(0).Length;
            int colSize = residual.GetRow(0).Length;
            double[] posterior = new double[rowSize];
            for (int i = 0; i < rowSize; i++) {
                for (int j = 0; j < colSize; j++) {
                    posterior[i] += kGain.Get(i, j) * residual.Get(0, j);
                }
                posterior[i] += x[i];
            }

            return posterior.ToList();
        }

        public Matrix GetPosteriorP(Matrix covariance, Matrix weightedCovariance, Matrix kGain) {
            Matrix P = covariance.Subtract(kGain.Multiply(weightedCovariance.Multiply(kGain.Transpose())));
            return P.Add(P.Transpose()).Multiply(0.5);
        }

        public Matrix TransitionSigmas(StateTransitionModel transitionModelFx, Matrix sigmaPointsX) {
            int rowSize = sigmaPointsX.GetColumn(0).Length;
            int colSize = sigmaPointsX.GetRow(0).Length;
            List<double> transformedSigmas, sigmas;
            var y = new Matrix(rowSize, colSize);
            for (int i = 0; i < rowSize; i++) {
                sigmas = sigmaPointsX.GetRow(i).ToList();
                transformedSigmas = transitionModelFx.Next(sigmas);
                y.SetRow(i, transformedSigmas.ToArray());
            }
            return y;
        }

        public void Predict() {
            X = sigmaGenerator.GetSigmaPoints(xPosterior, P_Posterior);
            
            // Y = f(x)
            Y = TransitionSigmas(stateTransitionModel, X);
            //xPosterior = [0.28708134 0.14354067 0.28708134 0.14354067]

            // xBar = Sum(wM * Y)
            xPrior = GetWeightedSigmas(meanWeight, Y);

            // _P = Sum(    wC(Y - _x)*(Y - _x) + Q   )
            P_Prior = GetWeightedCovariance(Y, xPrior, covarianceWeight).Add(Q);
        }

        public void Update(double[] measurement) {
            // Z = h(Y)
            Zeta = measurementSpace.Next(Y);

            // Uz = Sum(wM * Z)
            Uz = GetWeightedSigmas(meanWeight, Zeta);

            // y = z - uZ
            residual_y = GetResidual(measurement.ToList(), Uz);

            // Pz = Sum(    wC*(Z - uZ)*Transpose(Z - uZ)  + R  )
            Pz = GetWeightedCovariance(Zeta, Uz, covarianceWeight).Add(R);
            Pxz = GetCrossVariance(xPrior, Uz, covarianceWeight, Y, Zeta);
            K = GetKalmanGain(Pxz, Pz);

            // x = _x + K*y
            xPosterior = GetPosteriorX(xPrior, K, residual_y);

            // P = P - K*Pz*Transpose(K)
            P_Posterior = GetPosteriorP(P_Prior, Pz, K);
        }

        public void UnscentedTransform() {

        }

        public double[] GetArrayPos() {
            return xPosterior.ToArray();
        }

        public double GetPos() {
            throw new NotImplementedException();
        }

        public double GetVel() {
            throw new NotImplementedException();
        }
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
            if (!covariance.IsSymmetric()) {
                throw new ArgumentException("Covariance is not symmetric");
            }
            /*
            0.085226597003725035 0.074568888375718734                    0                    0
            0.074568888375718845 0.134950783708109030                    0                    0
                               0                    0 0.085226597003724591 0.074568888375718512
                               0                    0 0.074568888375718623 0.134950783708108870
            */

            //Create Identity if P is 1x1
            Matrix P = covariance;
            if (covariance.GetRow(0).Length == 1 && covariance.GetColumn(0).Length == 1) {
                P = CreateScaledIdentity(covariance.Get(0, 0));
            }

            int sigmaCount = 2 * n + 1;
            var sigmas = new Matrix(sigmaCount, n);
            double lambda = (a * a) * (n + k) - n;
            Matrix scaledP = P.Multiply(lambda * million + n * million).Multiply(1 / million);
            Matrix U = scaledP.Cholesky(true);

            for (int i = 0; i < sigmaCount; i++) {
                sigmas.SetRow(i, mean.ToArray());
            }

            for (int j = 0; j < sigmaCount / 2; j++) {
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
            for (int i = 0; i < (2 * n); i++) {
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
