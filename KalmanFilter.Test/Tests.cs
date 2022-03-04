using NUnit.Framework;
using RenzLibraries;
using System;
using System.Linq;
using System.Collections.Generic;

namespace KalmanFilter.Test
{
    public class Tests
    {
        [SetUp]
        public void Setup() { }

        [Test]
        public void UnscentedKalman_GetWeightedCovariance_CovarianceCorrectlyWeighted() {
            Matrix y = new Matrix(9, 4);
            y.SetRow(0, new double[] { 0, 0, 0, 0 });
            y.SetRow(1, new double[] { 0.2236068, 0, 0, 0 });
            y.SetRow(2, new double[] { 0, 0.2236068, 0, 0 });
            y.SetRow(3, new double[] { 0, 0, 0.2236068, 0 });
            y.SetRow(4, new double[] { 0, 0, 0, 0.2236068 });
            y.SetRow(5, new double[] { 0.2236068, 0, 0, 0 });
            y.SetRow(6, new double[] { 0, 0.2236068, 0, 0 });
            y.SetRow(7, new double[] { 0, 0, 0.2236068, 0 });
            y.SetRow(8, new double[] { 0, 0, 0, 0.2236068 });

            Matrix weightedSigmas = new Matrix(9, 4);
            weightedSigmas.SetRow(0, new double[] { 0, 0, 0, 0 });
            weightedSigmas.SetRow(1, new double[] { 0.2236068, 0, 0, 0 });
            weightedSigmas.SetRow(2, new double[] { 0.2236068, 0.2236068, 0, 0 });
            weightedSigmas.SetRow(3, new double[] { 0, 0, 0.2236068, 0 });
            weightedSigmas.SetRow(4, new double[] { 0, 0, 0.2236068, 0.2236068 });
            weightedSigmas.SetRow(5, new double[] { -0.2236068, 0, 0, 0 });
            weightedSigmas.SetRow(6, new double[] { -0.2236068, -0.2236068, 0, 0 });
            weightedSigmas.SetRow(7, new double[] { 0, 0, -0.2236068, 0 });
            weightedSigmas.SetRow(8, new double[] { 0, 0, -0.2236068, -0.2236068 });

            var xBar = new List<double>() { 0, 0, 0, 0 };
            var Wc = new List<double>() { -76.01, 10, 10, 10, 10, 10, 10, 10, 10 };
            Matrix Q = 
            /*
             Q
            [[0.005 0.01  0.    0.   ]
             [0.01  0.02  0.    0.   ]
             [0.    0.    0.005 0.01 ]
             [0.    0.    0.01  0.02 ]]

            mean = [0. 0. 0. 0.]
            P = [[1. 0. 0. 0.]
             [0. 1. 0. 0.]
             [0. 0. 1. 0.]
             [0. 0. 0. 1.]]
            x
            [[ 0.         0.         0.         0.       ]
             [ 0.2236068  0.         0.         0.       ]
             [ 0.         0.2236068  0.         0.       ]
             [ 0.         0.         0.2236068  0.       ]
             [ 0.         0.         0.         0.2236068]
             [-0.2236068  0.         0.         0.       ]
             [ 0.        -0.2236068  0.         0.       ]
             [ 0.         0.        -0.2236068  0.       ]
             [ 0.         0.         0.        -0.2236068]]
            args
            {}
            Y
            [[ 0.         0.         0.         0.       ]
             [ 0.2236068  0.         0.         0.       ]
             [ 0.2236068  0.2236068  0.         0.       ]
             [ 0.         0.         0.2236068  0.       ]
             [ 0.         0.         0.2236068  0.2236068]
             [-0.2236068  0.         0.         0.       ]
             [-0.2236068 -0.2236068  0.         0.       ]
             [ 0.         0.        -0.2236068  0.       ]
             [ 0.         0.        -0.2236068 -0.2236068]]
            sigmas [[ 0.         0.         0.         0.       ]
             [ 0.2236068  0.         0.         0.       ]
             [ 0.2236068  0.2236068  0.         0.       ]
             [ 0.         0.         0.2236068  0.       ]
             [ 0.         0.         0.2236068  0.2236068]
             [-0.2236068  0.         0.         0.       ]
             [-0.2236068 -0.2236068  0.         0.       ]
             [ 0.         0.        -0.2236068  0.       ]
             [ 0.         0.        -0.2236068 -0.2236068]]
            x [1.05007638e-16 1.05007638e-16 1.05007638e-16 1.05007638e-16]
            x[np.newaxis, :] [[1.05007638e-16 1.05007638e-16 1.05007638e-16 1.05007638e-16]]
            y [[-1.05007638e-16 -1.05007638e-16 -1.05007638e-16 -1.05007638e-16]
             [ 2.23606798e-01 -1.05007638e-16 -1.05007638e-16 -1.05007638e-16]
             [ 2.23606798e-01  2.23606798e-01 -1.05007638e-16 -1.05007638e-16]
             [-1.05007638e-16 -1.05007638e-16  2.23606798e-01 -1.05007638e-16]
             [-1.05007638e-16 -1.05007638e-16  2.23606798e-01  2.23606798e-01]
             [-2.23606798e-01 -1.05007638e-16 -1.05007638e-16 -1.05007638e-16]
             [-2.23606798e-01 -2.23606798e-01 -1.05007638e-16 -1.05007638e-16]
             [-1.05007638e-16 -1.05007638e-16 -2.23606798e-01 -1.05007638e-16]
             [-1.05007638e-16 -1.05007638e-16 -2.23606798e-01 -2.23606798e-01]]
            np.diag(Wc) [[-76.01   0.     0.     0.     0.     0.     0.     0.     0.  ]
             [  0.    10.     0.     0.     0.     0.     0.     0.     0.  ]
             [  0.     0.    10.     0.     0.     0.     0.     0.     0.  ]
             [  0.     0.     0.    10.     0.     0.     0.     0.     0.  ]
             [  0.     0.     0.     0.    10.     0.     0.     0.     0.  ]
             [  0.     0.     0.     0.     0.    10.     0.     0.     0.  ]
             [  0.     0.     0.     0.     0.     0.    10.     0.     0.  ]
             [  0.     0.     0.     0.     0.     0.     0.    10.     0.  ]
             [  0.     0.     0.     0.     0.     0.     0.     0.    10.  ]]
            np.dot(np.diag(Wc), y) [[ 7.98163058e-15  7.98163058e-15  7.98163058e-15  7.98163058e-15]
             [ 2.23606798e+00 -1.05007638e-15 -1.05007638e-15 -1.05007638e-15]
             [ 2.23606798e+00  2.23606798e+00 -1.05007638e-15 -1.05007638e-15]
             [-1.05007638e-15 -1.05007638e-15  2.23606798e+00 -1.05007638e-15]
             [-1.05007638e-15 -1.05007638e-15  2.23606798e+00  2.23606798e+00]
             [-2.23606798e+00 -1.05007638e-15 -1.05007638e-15 -1.05007638e-15]
             [-2.23606798e+00 -2.23606798e+00 -1.05007638e-15 -1.05007638e-15]
             [-1.05007638e-15 -1.05007638e-15 -2.23606798e+00 -1.05007638e-15]
             [-1.05007638e-15 -1.05007638e-15 -2.23606798e+00 -2.23606798e+00]]
            yT [[-1.05007638e-16  2.23606798e-01  2.23606798e-01 -1.05007638e-16
              -1.05007638e-16 -2.23606798e-01 -2.23606798e-01 -1.05007638e-16
              -1.05007638e-16]
             [-1.05007638e-16 -1.05007638e-16  2.23606798e-01 -1.05007638e-16
              -1.05007638e-16 -1.05007638e-16 -2.23606798e-01 -1.05007638e-16
              -1.05007638e-16]
             [-1.05007638e-16 -1.05007638e-16 -1.05007638e-16  2.23606798e-01
               2.23606798e-01 -1.05007638e-16 -1.05007638e-16 -2.23606798e-01
              -2.23606798e-01]
             [-1.05007638e-16 -1.05007638e-16 -1.05007638e-16 -1.05007638e-16
               2.23606798e-01 -1.05007638e-16 -1.05007638e-16 -1.05007638e-16
              -2.23606798e-01]]
            P [[2.00000000e+00 1.00000000e+00 1.14670370e-31 1.14670370e-31]
             [1.00000000e+00 1.00000000e+00 1.14670370e-31 1.14670370e-31]
             [8.13388247e-32 8.13388247e-32 2.00000000e+00 1.00000000e+00]
             [8.13388247e-32 8.13388247e-32 1.00000000e+00 1.00000000e+00]]
            Wm[-79.  10.  10.  10.  10.  10.  10.  10.  10.]
            Wc[-76.01  10.    10.    10.    10.    10.    10.    10.    10.  ]
            xBar = [1.05007638e-16 1.05007638e-16 1.05007638e-16 1.05007638e-16]
            PBar = [[2.00500000e+00 1.01000000e+00 1.14670370e-31 1.14670370e-31]
             [1.01000000e+00 1.02000000e+00 1.14670370e-31 1.14670370e-31]
             [8.13388247e-32 8.13388247e-32 2.00500000e+00 1.01000000e+00]
             [8.13388247e-32 8.13388247e-32 1.01000000e+00 1.02000000e+00]
             */
        }

        [Test]
        public void UnscentedKalman_TransitionSigmas_SigmasCorrectlyTransited() {
            UnscentedKalman UKF = new UnscentedKalman();

            Matrix F = new Matrix(4, 4);
            F.SetRow(0, new double[] { 1, 1, 0, 0 });
            F.SetRow(1, new double[] { 0, 1, 0, 0 });
            F.SetRow(2, new double[] { 0, 0, 1, 1 });
            F.SetRow(3, new double[] { 0, 0, 0, 1 });
            var stateTransition = new StateTransitionModel(F);

            Matrix x = new Matrix(9, 4);
            x.SetRow(0, new double[] { 0, 0, 0, 0 });
            x.SetRow(1, new double[] { 0.2236068, 0, 0, 0 });
            x.SetRow(2, new double[] { 0, 0.2236068, 0, 0 });
            x.SetRow(3, new double[] { 0, 0, 0.2236068, 0 });
            x.SetRow(4, new double[] { 0, 0, 0, 0.2236068 });
            x.SetRow(5, new double[] { 0.2236068, 0, 0, 0 });
            x.SetRow(6, new double[] { 0, 0.2236068, 0, 0 });
            x.SetRow(7, new double[] { 0, 0, 0.2236068, 0 });
            x.SetRow(8, new double[] { 0, 0, 0, 0.2236068 });

            Matrix expectedSigmas = new Matrix(9, 4);
            expectedSigmas.SetRow(0, new double[] { 0, 0, 0, 0 });
            expectedSigmas.SetRow(1, new double[] { 0.2236068, 0, 0, 0 });
            expectedSigmas.SetRow(2, new double[] { 0.2236068, 0.2236068, 0, 0 });
            expectedSigmas.SetRow(3, new double[] { 0, 0, 0.2236068, 0 });
            expectedSigmas.SetRow(4, new double[] { 0, 0, 0.2236068, 0.2236068 });
            expectedSigmas.SetRow(5, new double[] { -0.2236068, 0, 0, 0 });
            expectedSigmas.SetRow(6, new double[] { -0.2236068, -0.2236068, 0, 0 });
            expectedSigmas.SetRow(7, new double[] { 0, 0, -0.2236068, 0 });
            expectedSigmas.SetRow(8, new double[] { 0, 0, -0.2236068, -0.2236068 });

            Matrix actualSigmas = UKF.TransitionSigmas(stateTransition, x);

            for (int i = 0; i < x.GetRow(0).Length; i++) {
                Assert.AreEqual(expectedSigmas.GetRow(i), actualSigmas.GetRow(i).Select(s => Math.Round(s, 7)));
            }
        }

        [Test]
        public void UnscentedKalman_Predict_CorrectFirstPrediction() {
            var mean = new List<double>() { 0, 0, 0, 0 };
            Matrix P = new Matrix(4, 4);
            P.SetRow(0, new double[] { 1, 0, 0, 0 });
            P.SetRow(1, new double[] { 0, 1, 0, 0 });
            P.SetRow(2, new double[] { 0, 0, 1, 0 });
            P.SetRow(3, new double[] { 0, 0, 0, 1 });
            Matrix F = new Matrix(4, 4);
            F.SetRow(0, new double[] { 1, 1, 0, 0 });
            F.SetRow(1, new double[] { 0, 1, 0, 0 });
            F.SetRow(2, new double[] { 0, 0, 1, 1 });
            F.SetRow(3, new double[] { 0, 0, 0, 1 });
            var sigmaPointGenerator = new MerweScaledSigmaPoints(4, 0.1, 2, 1);
            var stateTransition = new StateTransitionModel(F);
            var measurementSpace = new MeasurementSpace();
            var Q = new Matrix();
            UnscentedKalman UKF = new UnscentedKalman(mean, P, sigmaPointGenerator, stateTransition, measurementSpace,Q);
            UKF.Predict(mean, P, 1);
            Assert.IsTrue(false);
        }
        /*
        mean = [0. 0. 0. 0.]
        P = [[1. 0. 0. 0.]
         [0. 1. 0. 0.]
         [0. 0. 1. 0.]
         [0. 0. 0. 1.]]
        x
        [[ 0.         0.         0.         0.       ]
         [ 0.2236068  0.         0.         0.       ]
         [ 0.         0.2236068  0.         0.       ]
         [ 0.         0.         0.2236068  0.       ]
         [ 0.         0.         0.         0.2236068]
         [-0.2236068  0.         0.         0.       ]
         [ 0.        -0.2236068  0.         0.       ]
         [ 0.         0.        -0.2236068  0.       ]
         [ 0.         0.         0.        -0.2236068]]
        Y
        [[ 0.         0.         0.         0.       ]
         [ 0.2236068  0.         0.         0.       ]
         [ 0.2236068  0.2236068  0.         0.       ]
         [ 0.         0.         0.2236068  0.       ]
         [ 0.         0.         0.2236068  0.2236068]
         [-0.2236068  0.         0.         0.       ]
         [-0.2236068 -0.2236068  0.         0.       ]
         [ 0.         0.        -0.2236068  0.       ]
         [ 0.         0.        -0.2236068 -0.2236068]]
        Wm
        [-79.  10.  10.  10.  10.  10.  10.  10.  10.]
        Wc
        [-76.01  10.    10.    10.    10.    10.    10.    10.    10.  ]
        xBar = [1.05007638e-16 1.05007638e-16 1.05007638e-16 1.05007638e-16]
        PBar = [[2.00500000e+00 1.01000000e+00 1.14670370e-31 1.14670370e-31]
         [1.01000000e+00 1.02000000e+00 1.14670370e-31 1.14670370e-31]
         [8.13388247e-32 8.13388247e-32 2.00500000e+00 1.01000000e+00]
         [8.13388247e-32 8.13388247e-32 1.01000000e+00 1.02000000e+00]]
         */
        [Test]
        public void UnscentedKalman_GetWeightedSigmas_SigmasCorrectlyWeighted() {
            var meanWeights = new List<double>() { -79, 10, 10, 10, 10, 10, 10, 10, 10 };
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
            List<double> actualSigmas = UKF.GetWeightedSigmas(meanWeights, sigmasY);
            var expectedSigmas = new double[] { 0.43062201, 0.14354067, 0.43062201, 0.14354067 };
            Assert.AreEqual(    expectedSigmas, actualSigmas.Select(s => Math.Round(s,8))   );
        }
        /*
        mean = [0.28708134 0.14354067 0.28708134 0.14354067]
        P = [[ 9.11244019e-02  5.30622010e-02  1.62456105e-32 -7.60840380e-32]
         [ 5.30622010e-02  5.41531100e-01  1.82773143e-32 -4.29717191e-33]
         [-1.70859351e-32 -1.50542313e-32  9.11244019e-02  5.30622010e-02]
         [-1.09415584e-31 -3.76287175e-32  5.30622010e-02  5.41531100e-01]]
        x
        [[ 0.28708134  0.14354067  0.28708134  0.14354067]
         [ 0.35458112  0.18284613  0.28708134  0.14354067]
         [ 0.28708134  0.30332689  0.28708134  0.14354067]
         [ 0.28708134  0.14354067  0.35458112  0.18284613]
         [ 0.28708134  0.14354067  0.28708134  0.30332689]
         [ 0.21958156  0.10423521  0.28708134  0.14354067]
         [ 0.28708134 -0.01624555  0.28708134  0.14354067]
         [ 0.28708134  0.14354067  0.21958156  0.10423521]
         [ 0.28708134  0.14354067  0.28708134 -0.01624555]]
        args
        {}
        Y
        [[ 0.43062201  0.14354067  0.43062201  0.14354067]
         [ 0.53742725  0.18284613  0.43062201  0.14354067]
         [ 0.59040823  0.30332689  0.43062201  0.14354067]
         [ 0.43062201  0.14354067  0.53742725  0.18284613]
         [ 0.43062201  0.14354067  0.59040823  0.30332689]
         [ 0.32381677  0.10423521  0.43062201  0.14354067]
         [ 0.27083579 -0.01624555  0.43062201  0.14354067]
         [ 0.43062201  0.14354067  0.32381677  0.10423521]
         [ 0.43062201  0.14354067  0.27083579 -0.01624555]]
        Wm[-79.  10.  10.  10.  10.  10.  10.  10.  10.]
        Wc[-76.01  10.    10.    10.    10.    10.    10.    10.    10.  ]
        xBar = [0.43062201 0.14354067 0.43062201 0.14354067]
        PBar = [[ 7.43779904e-01  6.04593301e-01  5.00926675e-29 -1.81438008e-29]
         [ 6.04593301e-01  5.61531100e-01 -9.13353017e-30  3.67313359e-30]
         [ 5.25241290e-29 -9.43866348e-30  7.43779904e-01  6.04593301e-01]
         [-1.72900611e-29  3.36800028e-30  6.04593301e-01  5.61531100e-01]]
         */


        [Test]
        public void StateTransitionModel_Next_CorrectStateValues() {
            Matrix F = new Matrix(4, 4);
            F.SetRow(0, new double[] { 1, 1, 0, 0 });
            F.SetRow(1, new double[] { 0, 1, 0, 0 });
            F.SetRow(2, new double[] { 0, 0, 1, 1 });
            F.SetRow(3, new double[] { 0, 0, 0, 1 });
            StateTransitionModel model = new StateTransitionModel(F);

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
            Matrix identity = sigma.CreateScalarIdentity(scalar);
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
