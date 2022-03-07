using NUnit.Framework;
using RenzLibraries;
using System;
using System.Linq;
using System.Collections.Generic;

namespace KalmanFilter.Test
{
    public class Tests {

        List<double> xBar_1, Uz_1, posteriorX_1, meanWeight_1, covarianceWeight_1, measurementZ_1;
        Matrix X_1, Y_1, Q_1, F_1, pBar_1, zeta_1, residualY_1, Pz_1, Pxz_1, K_1, posteriorP_1;
        int dp;

        /*
        To Do:
        - Test GetResidual()
        */

        [SetUp]
        public void Setup() {

            //Expected Values for Unscented Kalman Filter
            Uz_1 = new List<double>() { 0, 0 };
            xBar_1 = new List<double>() { 0, 0, 0, 0 };
            measurementZ_1 = new List<double>() { 0.3, 0.3 };
            meanWeight_1 = new List<double>() { -79, 10, 10, 10, 10, 10, 10, 10, 10 };
            covarianceWeight_1 = new List<double>() { -76.01, 10, 10, 10, 10, 10, 10, 10, 10 };
            posteriorX_1 = new List<double>() { 0.28708134, 0.14354067, 0.28708134, 0.14354067 };

            X_1 = new Matrix(9, 4);
            X_1.SetRow(0, new double[] { 0, 0, 0, 0 });
            X_1.SetRow(1, new double[] { 0.2236068, 0, 0, 0 });
            X_1.SetRow(2, new double[] { 0, 0.2236068, 0, 0 });
            X_1.SetRow(3, new double[] { 0, 0, 0.2236068, 0 });
            X_1.SetRow(4, new double[] { 0, 0, 0, 0.2236068 });
            X_1.SetRow(5, new double[] { 0.2236068, 0, 0, 0 });
            X_1.SetRow(6, new double[] { 0, 0.2236068, 0, 0 });
            X_1.SetRow(7, new double[] { 0, 0, 0.2236068, 0 });
            X_1.SetRow(8, new double[] { 0, 0, 0, 0.2236068 });

            Y_1 = new Matrix(9, 4);
            Y_1.SetRow(0, new double[] { 0, 0, 0, 0 });
            Y_1.SetRow(1, new double[] { 0.2236068, 0, 0, 0 });
            Y_1.SetRow(2, new double[] { 0.2236068, 0.2236068, 0, 0 });
            Y_1.SetRow(3, new double[] { 0, 0, 0.2236068, 0 });
            Y_1.SetRow(4, new double[] { 0, 0, 0.2236068, 0.2236068 });
            Y_1.SetRow(5, new double[] { -0.2236068, 0, 0, 0 });
            Y_1.SetRow(6, new double[] { -0.2236068, -0.2236068, 0, 0 });
            Y_1.SetRow(7, new double[] { 0, 0, -0.2236068, 0 });
            Y_1.SetRow(8, new double[] { 0, 0, -0.2236068, -0.2236068 });

            F_1 = new Matrix(4, 4);
            F_1.SetRow(0, new double[] { 1, 1, 0, 0 });
            F_1.SetRow(1, new double[] { 0, 1, 0, 0 });
            F_1.SetRow(2, new double[] { 0, 0, 1, 1 });
            F_1.SetRow(3, new double[] { 0, 0, 0, 1 });

            pBar_1 = new Matrix(4, 4);
            pBar_1.SetRow(0, new double[] { 2.005, 1.01, 0, 0 });
            pBar_1.SetRow(1, new double[] { 1.01, 1.02, 0, 0 });
            pBar_1.SetRow(2, new double[] { 0, 0, 2.005, 1.01 });
            pBar_1.SetRow(3, new double[] { 0, 0, 1.01, 1.02 });

            zeta_1 = new Matrix(9, 2);
            zeta_1.SetRow(0, new double[] { 0, 0 });
            zeta_1.SetRow(1, new double[] { 0.2236068, 0 });
            zeta_1.SetRow(2, new double[] { 0.2236068, 0 });
            zeta_1.SetRow(3, new double[] { 0, 0.2236068 });
            zeta_1.SetRow(4, new double[] { 0, 0.2236068 });
            zeta_1.SetRow(5, new double[] { -0.2236068, 0 });
            zeta_1.SetRow(6, new double[] { -0.2236068, 0 });
            zeta_1.SetRow(7, new double[] { 0, -0.2236068 });
            zeta_1.SetRow(8, new double[] { 0, -0.2236068 });
            
            residualY_1 = new Matrix(1, 2);
            residualY_1.SetRow(0, new double[] { 0.3, 0.3 });

            Pz_1 = new Matrix(2, 2);
            Pz_1.SetRow(0, new double[] { 2.09, 0 });
            Pz_1.SetRow(1, new double[] { 0, 2.09 });

            Pxz_1 = new Matrix(4, 2);
            Pxz_1.SetRow(0, new double[] { 2, 0 });
            Pxz_1.SetRow(1, new double[] { 1, 0 });
            Pxz_1.SetRow(2, new double[] { 0, 2 });
            Pxz_1.SetRow(3, new double[] { 0, 1 });

            K_1 = new Matrix(4, 2);
            K_1.SetRow(0, new double[] { 0.9569378, 0 });
            K_1.SetRow(1, new double[] { 0.4784689, 0 });
            K_1.SetRow(2, new double[] { 0, 0.9569378 });
            K_1.SetRow(3, new double[] { 0, 0.4784689 });
            
            posteriorP_1 = new Matrix(4, 4);
            posteriorP_1.SetRow(0, new double[] { 0.09112440, 0.05306220, 0, 0 });
            posteriorP_1.SetRow(1, new double[] { 0.05306220, 0.5415311, 0, 0 });
            posteriorP_1.SetRow(2, new double[] { 0, 0, 0.09112440, 0.05306220 });
            posteriorP_1.SetRow(3, new double[] { 0, 0, 0.05306220, 0.5415311 });

            Q_1 = new Matrix(4, 4);
            Q_1.SetRow(0, new double[] { 0.005, 0.01, 0, 0 });
            Q_1.SetRow(1, new double[] { 0.01, 0.02, 0, 0 });
            Q_1.SetRow(2, new double[] { 0, 0, 0.005, 0.01 });
            Q_1.SetRow(3, new double[] { 0, 0, 0.01, 0.02 });
        }   

        [Test]
        public void UnscentedKalman_GetResidual_CorrectResidual() {
            UnscentedKalman UKF = new UnscentedKalman();
            Matrix Y = UKF.GetResidual(measurementZ_1, Uz_1);
            int rowSize = residualY_1.GetColumn(0).Length;
            for (int i = 0; i < rowSize; i++) {
                Assert.AreEqual(residualY_1.GetRow(i), Y.GetRow(i).Select( x => Math.Round(x, 3)));
            }
        }

        [Test]
        public void UnscentedKalman_GetPosteriorP_CorrectPosteriorForP() {
            UnscentedKalman UKF = new UnscentedKalman(); 
            Matrix P_posterior = UKF.GetPosteriorP(pBar_1, Pz_1, K_1);
            int rowSize = posteriorP_1.GetColumn(0).Length;
            for (int i = 0; i < rowSize; i++) {
                Assert.AreEqual(posteriorP_1.GetRow(i), P_posterior.GetRow(i).Select(p => Math.Round(p, 8)));
            }
        }

        [Test]
        public void UnscentedKalman_GetPosteriorX_CorrectPosteriorForX() {
            UnscentedKalman UKF = new UnscentedKalman();
            List<double> x_posterior = UKF.GetPosteriorX(xBar_1, K_1, residualY_1).Select(x => Math.Round(x,8)).ToList();
            Assert.AreEqual(posteriorX_1, x_posterior);
        }

        [Test]
        public void UnscentedKalman_GetKalmanGain_CorrectKalman() {
            UnscentedKalman UKF = new UnscentedKalman();
            Matrix kalman = UKF.GetKalmanGain(Pxz_1, Pz_1);

            int rowSize = K_1.GetColumn(0).Length;
            for (int i = 0; i < rowSize; i++) {
                Assert.AreEqual(K_1.GetRow(i), kalman.GetRow(i).Select(k => Math.Round(k, 7)));
            }
        }

        [Test]
        public void UnscentedKalman_GetCrossVariance_CorrectVariance() {
            UnscentedKalman UKF = new UnscentedKalman();
            Matrix Pxz = UKF.GetCrossVariance(xBar_1, Uz_1, meanWeight_1, Y_1, zeta_1);

            int rowSize = Pxz_1.GetColumn(0).Length;
            for (int i = 0; i < rowSize; i++) {
                Assert.AreEqual(Pxz_1.GetRow(i), Pxz.GetRow(i).Select(x => Math.Round(x, 7)));
            }
        }

        [Test]
        public void UnscentedKalman_GetWeightedCovariance_CovarianceCorrectlyWeighted() {
            UnscentedKalman UKF = new UnscentedKalman();
            Matrix Pbar = UKF.GetWeightedCovariance(Y_1, xBar_1, covarianceWeight_1).Add(Q_1);

            int rowSize = pBar_1.GetColumn(0).Length;
            for (int i = 0; i < rowSize; i++) {
                Assert.AreEqual(pBar_1.GetRow(i), Pbar.GetRow(i).Select(p => Math.Round(p, 3)));
            }
        }

        [Test]
        public void UnscentedKalman_TransitionSigmas_SigmasCorrectlyTransformed() {
            UnscentedKalman UKF = new UnscentedKalman();
            var stateTransition = new StateTransitionModel(F_1);
            Matrix actualSigmas = UKF.TransitionSigmas(stateTransition, X_1);

            for (int i = 0; i < X_1.GetRow(0).Length; i++) {
                Assert.AreEqual(Y_1.GetRow(i), actualSigmas.GetRow(i).Select(s => Math.Round(s, 7)));
            }
        }

        [Test]
        public void UnscentedKalman_GetWeightedSigmas_SigmasCorrectlyWeighted() {
            var meanWeights_1 = new List<double>() { -79, 10, 10, 10, 10, 10, 10, 10, 10 };
            var sigmasY = new Matrix(9, 4);
            sigmasY.SetRow(0, new double[] { 0.43062201, 0.14354067, 0.43062201, 0.14354067 });
            sigmasY.SetRow(1, new double[] { 0.53742725, 0.18284613, 0.43062201, 0.14354067 });
            sigmasY.SetRow(2, new double[] { 0.59040823, 0.30332689, 0.43062201, 0.14354067 });
            sigmasY.SetRow(3, new double[] { 0.43062201, 0.14354067, 0.53742725, 0.18284613 });
            sigmasY.SetRow(4, new double[] { 0.43062201, 0.14354067, 0.59040823, 0.30332689 });
            sigmasY.SetRow(5, new double[] { 0.32381677, 0.10423521, 0.43062201, 0.14354067 });
            sigmasY.SetRow(6, new double[] { 0.27083579,-0.01624555, 0.43062201, 0.14354067 });
            sigmasY.SetRow(7, new double[] { 0.43062201, 0.14354067, 0.32381677, 0.10423521 });
            sigmasY.SetRow(8, new double[] { 0.43062201, 0.14354067, 0.27083579,-0.01624555 });

            UnscentedKalman UKF = new UnscentedKalman();
            List<double> actualSigmas = UKF.GetWeightedSigmas(meanWeights_1, sigmasY);
            var expectedSigmas = new double[] { 0.43062201, 0.14354067, 0.43062201, 0.14354067 };
            Assert.AreEqual(    expectedSigmas, actualSigmas.Select(s => Math.Round(s,8))   );
        }

        [Test]
        public void StateTransitionModel_Next_CorrectStateValues() {
            StateTransitionModel model = new StateTransitionModel(F_1);
            var state = new List<double>() { 5, 7, 11, 13 };
            List<double> newState = model.Next(state);
            Assert.AreEqual(new List<double> { 12, 7, 24, 13}, newState);
        }

        [Test]
        public void MerweScaledSigmaPoints_MeanWeight_CorrectLength() {
            int dimension = 4;
            double alpha = 0.1, beta = 2.0, kappa = 1.0;
            MerweScaledSigmaPoints sigma = new MerweScaledSigmaPoints(dimension, alpha, beta, kappa);
            List<double> weight = sigma.GetMeanWeight();
            Assert.AreEqual(2 * dimension + 1, weight.Count);
        }

        [Test]
        public void MerweScaledSigmaPoints_CovarianceWeight_CorrectLength() {
            int dimension = 4;
            double alpha = 0.1, beta = 2.0, kappa = 1.0;
            MerweScaledSigmaPoints sigma = new MerweScaledSigmaPoints(dimension, alpha, beta, kappa);
            List<double> weight = sigma.GetCovarianceWeight();
            Assert.AreEqual(2 * dimension + 1, weight.Count);
        }

        [Test]
        public void MerweScaledSigmaPoints_MeanWeight_CorrectWeights() {
            int n = 2; double alpha = 0.09, beta = 2, kappa = 1;
            MerweScaledSigmaPoints sigma = new MerweScaledSigmaPoints(n, alpha, beta, kappa);
            List<double> weight = sigma.GetMeanWeight();
            Assert.AreEqual(new List<double>() { -81.305, 20.576, 20.576, 20.576, 20.576 }, weight.Select(w => Math.Round(w, 3)));

            n = 2; alpha = 0.15; beta = 1; kappa = 0.15;
            sigma = new MerweScaledSigmaPoints(n, alpha, beta, kappa);
            weight = sigma.GetMeanWeight();
            Assert.AreEqual(new List<double>() { -40.344, 10.336, 10.336, 10.336, 10.336 }, weight.Select(w => Math.Round(w, 3)));

            n = 4; alpha = 0.1; beta = 2.0; kappa = 1.0;
            sigma = new MerweScaledSigmaPoints(n, alpha, beta, kappa);
            weight = sigma.GetMeanWeight();
            Assert.AreEqual(new List<double>() { -79, 10, 10, 10, 10, 10, 10, 10, 10 }, weight.Select(w => Math.Round(w, 3)));
        }

        [Test]
        public void MerweScaledSigmaPoints_CovarianceWeight_CorrectWeights() {
            int n = 2; double alpha = 0.09, beta = 2, kappa = 1;
            MerweScaledSigmaPoints sigma = new MerweScaledSigmaPoints(n, alpha, beta, kappa);
            List<double> weight = sigma.GetCovarianceWeight();
            Assert.AreEqual(new List<double>() { -78.313, 20.576, 20.576, 20.576, 20.576 }, weight.Select(w => Math.Round(w, 3)));

            n = 2; alpha = 0.15; beta = 1; kappa = 0.15;
            sigma = new MerweScaledSigmaPoints(n, alpha, beta, kappa);
            weight = sigma.GetCovarianceWeight();
            Assert.AreEqual(new List<double>() { -38.366, 10.336, 10.336, 10.336, 10.336 }, weight.Select(w => Math.Round(w, 3)));

            n = 4; alpha = 0.1; beta = 2.0; kappa = 1.0;
            sigma = new MerweScaledSigmaPoints(n, alpha, beta, kappa);
            weight = sigma.GetCovarianceWeight();
            Assert.AreEqual(new List<double>() { -76.01, 10, 10, 10, 10, 10, 10, 10, 10 }, weight.Select(w => Math.Round(w, 3)));
        }

        [Test]
        public void MerweScaledSigmaPoints_CreateScalarIdentity_ValuesInDiagonalAreTheSame() {
            double scalar = 4;
            int n = 2; double alpha = 0.3, beta = 2, kappa = 0.1;
            MerweScaledSigmaPoints sigma = new MerweScaledSigmaPoints(n, alpha, beta, kappa);
            Matrix identity = sigma.CreateScaledIdentity(scalar);
            Assert.AreEqual(identity.GetRow(0).Length, n);
            Assert.AreEqual(identity.GetColumn(0).Length, n);
            Assert.AreEqual(identity.Get(0,0), scalar);
            Assert.AreEqual(identity.Get(1,1), scalar);
        }

        [Test]
        public void MerweScaledSigmaPoints_CalculateSigmaPoints_CorrectNumberOfSigmas() {
            int n = 2; double alpha = 0.3, beta = 2, kappa = 0.1;
            var mean = new List<double>() { 0, 0 };
            Matrix P = new Matrix(2, 2);
            P.SetRow(0, new double[] { 32, 15 });
            P.SetRow(1, new double[] { 15, 40 });

            MerweScaledSigmaPoints sigmaFunction = new MerweScaledSigmaPoints(n, alpha, beta, kappa);
            Matrix sigmaPoints = sigmaFunction.GetSigmaPoints(mean, P);
            Assert.AreEqual(5, sigmaPoints.GetColumn(0).Length);
        }

        [Test]
        public void MerweScaledSigmaPoints_CalculateSigmaPoints_CorrectSigmasForZeroMean() {
            var mean = new List<double>() { 0, 0 };
            int dimension = 2; 
            double alpha = 0.3, beta = 2, kappa = 0.1;
            Matrix P = new Matrix(2, 2);
            P.SetRow(0, new double[] { 32, 15 });
            P.SetRow(1, new double[] { 15, 40 });
            MerweScaledSigmaPoints sigmaFunction = new MerweScaledSigmaPoints(dimension, alpha, beta, kappa);
            Matrix sigmaPoints = sigmaFunction.GetSigmaPoints(mean, P);
            Assert.AreEqual(new double[] { 0, 0 }, sigmaPoints.GetRow(0).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 2.459, 1.153 }, sigmaPoints.GetRow(1).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 0, 2.496 }, sigmaPoints.GetRow(2).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { -2.459, -1.153 }, sigmaPoints.GetRow(3).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 0, -2.496 }, sigmaPoints.GetRow(4).Select(p => Math.Round(p, 3)));
        }

        [Test]
        public void MerweScaledSigmaPoints_CalculateSigmaPoints_CorrectSigmasForGivenMean() {
            var mean = new List<double>() { 0, 5 };
            int dimension = 2; 
            double alpha = 0.5, beta = 2, kappa = 0;
            Matrix P = new Matrix(2, 2);
            P.SetRow(0, new double[] { 4, -2.2 });
            P.SetRow(1, new double[] { -2.2, 3 });
            MerweScaledSigmaPoints sigmaFunction = new MerweScaledSigmaPoints(dimension, alpha, beta, kappa);
            Matrix sigmaPoints = sigmaFunction.GetSigmaPoints(mean, P);
            Assert.AreEqual(new double[] { 0, 5 }, sigmaPoints.GetRow(0).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 1.414, 4.222 }, sigmaPoints.GetRow(1).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 0, 5.946 }, sigmaPoints.GetRow(2).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { -1.414, 5.778 }, sigmaPoints.GetRow(3).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 0, 4.054 }, sigmaPoints.GetRow(4).Select(p => Math.Round(p, 3)));
        }

        [Test]
        public void MerweScaledSigmaPoints_CalculateSigmaPoints_CorrectSigmasForFourDimensions()
        {
            int dimension = 4; 
            double alpha = 0.1, beta = 2, kappa = -1;
            var mean = new List<double>() { 33511.251, 99.358, 1356.363, -0.85 };
            Matrix P = new Matrix(4, 4);
            P.SetRow(0, new double[] { 3683.71, 15.064, -1998.956, -0.022 });
            P.SetRow(1, new double[] { 15.064, 2.399, 0.172, 0 });
            P.SetRow(2, new double[] { -1998.956, 0.172, 21117.935, 15.607 });
            P.SetRow(3, new double[] { -0.022, 0, 15.607, 2.4 });
            MerweScaledSigmaPoints sigmaFunction = new MerweScaledSigmaPoints(dimension, alpha, beta, kappa);
            Matrix sigmaPoints = sigmaFunction.GetSigmaPoints(mean, P);
            Assert.AreEqual(new double[] { 33511.251, 99.358, 1356.363, -0.85 }, sigmaPoints.GetRow(0).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 33521.763, 99.401, 1350.658, -0.85 }, sigmaPoints.GetRow(1).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 33511.251, 99.623, 1357.309, -0.85 }, sigmaPoints.GetRow(2).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 33511.251, 99.358, 1380.86, -0.831 }, sigmaPoints.GetRow(3).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 33511.251, 99.358, 1356.363,-0.582 }, sigmaPoints.GetRow(4).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 33500.739, 99.315, 1362.068, -0.85 }, sigmaPoints.GetRow(5).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 33511.251, 99.093, 1355.417, -0.85 }, sigmaPoints.GetRow(6).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 33511.251, 99.358, 1331.866,-0.869 }, sigmaPoints.GetRow(7).Select(p => Math.Round(p, 3)));
            Assert.AreEqual(new double[] { 33511.251, 99.358, 1356.363,-1.118 }, sigmaPoints.GetRow(8).Select(p => Math.Round(p, 3)));
        }

        [Test]
        public void KalmanYOLOL_MatchesKalmanSimple() {
            double init_x = 0.1;
            double init_v = 0.2;
            double time = 1;
            double accel = 0.01;
            double Px = 0.01;
            double Pv = 0.01;
            double ObvError = 20;
            double[] measurements = new double[] { 10, 11, 12 };

            KalmanSimple kf_simple = new KalmanSimple(init_x, init_v, time, accel, Px, Pv, ObvError);
            KalmanYOLOL kf_yolol = new KalmanYOLOL(init_x, init_v, time, accel, Px, Pv, ObvError);

            foreach (var z in measurements) {
                kf_simple.Predict();
                kf_simple.Update(new double[] { z });

                kf_yolol.Predict();
                kf_yolol.Update(new double[] { z });
            }

            Assert.AreEqual(kf_simple.GetPos(), kf_yolol.GetPos());
            Assert.AreEqual(kf_simple.GetVel(), kf_yolol.GetVel());
        }

        [Test] 
        public void KalmanBowtie_Predicts2ndIteration() {
            KalmanBowtie kf = new KalmanBowtie(4000, 280, 2, 1, 20, 5, 25, 6);
            kf.Predict();
            kf.Update(new double[] { 4260, 282 });

            //1st iteration
            double pos1 = Math.Round(kf.GetPos(), 1);
            double vel1 = Math.Round(kf.GetVel(), 1);
            Assert.AreEqual(pos1, 4272.5);
            Assert.AreEqual(vel1, 282);

            //2nd iteration
            kf.Predict();
            kf.Update(new double[] { 4550, 285 });
            double pos2 = Math.Round(kf.GetPos(), 1);
            double vel2 = Math.Round(kf.GetVel(), 1);
            Assert.AreEqual(pos2, 4553.9);
            Assert.AreEqual(vel2, 284.3);
        }

        [Test]
        public void Kalman_Update_AssignsMeanAndPos() {
            double x = 0.2;
            double v = 2.3;
            double a = 1.2;
            double t = 1;
            double[] z = new double[] { 0.1 };
            double z_var = 0.1;
            Kalman kf = new Kalman(x, v, a, t, z_var);
            kf.Update(z);
            Assert.NotNull(kf.GetPos());
            Assert.NotNull(kf.Mean());
        }

        [Test]
        public void Kalman_Mean_CorrectSize() {
            double x = 0.2;
            double v = 2.3;
            double a = 1.2;
            double t = 1;
            double[] z = new double[] { 0.1 };
            double z_var = 0.1;
            Kalman kf = new Kalman(x, v, a, t, z_var);

            Assert.AreEqual(kf.Cov().GetRow(0).Length, 2);
            Assert.AreEqual(kf.Cov().GetColumn(0).Length, 2);
            Assert.AreEqual(kf.Mean().GetRow(0).Length, 1);
            Assert.AreEqual(kf.Mean().GetColumn(0).Length, 2);
        }

        [Test]
        public void Kalman_Update_StateUncertaintyIncreases() {
            double x = 0.2;
            double v = 2.3;
            double a = 1.2;
            double t = 1;
            double[] z = new double[] { 0.1 };
            double z_var = 0.1;
            Kalman kf = new Kalman(x, v, a, t, z_var);

            double det_before = kf.Cov().Determinant();//Matrix.Determinant(kf.Cov().GetDouble());
            kf.Update(z);
            double det_after = kf.Cov().Determinant();

            Assert.Less(det_after, det_before);
        }

        [Test]
        public void Kalman_Predict_StateUncertaintyIncreases() {
            double x = 0.2;
            double v = 2.3;
            double a = 1.2;
            double t = 1;
            double[] z = new double[] { 0.1 };
            double z_var = 0.1;
            Kalman kf = new Kalman(x, v, a, t, z_var);

            for (int i = 0; i < 10; i++) {
                double det_before = kf.Cov().Determinant();
                kf.Predict();
                double det_after = kf.Cov().Determinant();
                Assert.Greater(det_after, det_before);
            }
        }

        [Test]
        public void Kalman_GetVel_ReturnsValue() {
            double x = 0.2;
            double v = 2.3;
            double a = 1.2;
            double t = 1;
            double[] z = new double[] { 0.1 };
            double z_var = 0.1;
            Kalman kf = new Kalman(x, v, a, t, z_var);

            Assert.AreEqual(kf.GetPos(), x);
            Assert.AreEqual(kf.GetVel(), v);
        }
    }
}
