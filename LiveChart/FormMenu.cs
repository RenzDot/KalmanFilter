using KalmanFilter;
using Isan;
using LiveCharts;
using LiveCharts.Wpf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using RenzLibraries;
using static System.Math;
using LiveCharts.Defaults;

namespace Hamilton
{    
    public partial class FormMenu : Form {
        public static double[] raw_data = new double[] { 999993.937, 999992.812, 999988.687, 999979.125, 999964.062, 999946.0, 999922.312, 999894.812, 999867.062, 999835.062, 999801.187, 999769.5, 999736.375, 999698.875, 999660.75, 999618.875, 999583.437, 999540.687, 999501.25, 999465.25, 999435.812, 999386.312, 999342.625, 999302.625, 999262.937, 999226.562, 999186.437, 999146.125, 999106.187, 999063.25, 999023.187, 998982.937, 998942.937, 998902.937, 998866.312, 998829.937, 998790.0, 998750.125, 998706.5, 998666.375, 998629.937, 998593.437, 998549.687, 998509.125, 998469.437, 998432.437, 998392.75, 998352.75, 998309.312, 998268.812, 998228.062, 998188.75, 998151.062, 998111.5, 998072.187, 998031.062, 997994.625, 997954.187, 997914, 997874.937, 997831.375, 997794.812, 997755.687, 997712.062, 997675.375, 997631.437, 997591.875, 997550.875, 997508.312, 997468.187, 997428, 997391.5, 997352.187, 997314.812, 997274.625, 997234.5, 997194.437, 997157.937, 997117.812, 997077.875, 997041.75, 997005.437, 996964.375, 996924.062, 996883.875, 996847.375, 996807.187, 996766.625, 996726.375, 996685, 996648.75, 996608.5,        };        public static double[] raw_r1 = new double[] { 949378.125, 949378.000, 949378.000, 949377.437, 949376.062, 949373.750, 949370.500, 949370.500, 949366.375, 949361.437, 949355.687, 949349.375, 949342.500, 949342.500, 949335.125, 949327.312, 949319.187, 949310.750, 949310.750, 949302.062, 949293.187, 949284.125, 949274.875, 949274.875, 949265.562, 949256.125, 949246.625, 949237.000, 949237.000, 949227.375, 949217.750, 949208.000, 949198.187, 949188.375, 949188.375, 949178.562, 949168.750, 949158.875, 949149.000, 949149.000, 949139.125, 949129.250, 949119.375, 949109.500, 949109.500, 949099.562, 949089.625, 949079.687, 949069.750, 949059.750, 949059.750, 949049.812, 949039.875, 949029.875, 949029.875, 949019.875, 949009.875, 948999.937, 948989.937, 948980.000, 948980.000, 948970.000, 948960.062, 948950.000, 948939.937, 948939.937, 948929.937, 948919.937, 948909.875, 948899.875, 948889.875, 948889.875, 948879.812, 948869.812, 948859.750, 948849.750, 948849.750, 948839.687, 948829.625, 948819.625, 948809.562, 948809.562, 948799.500, 948789.437, 948779.375, 948769.312, 948759.250, 948759.250, 948749.187, 948739.062, 948729.000, 948718.875, 948718.875, 948708.812, 948698.687, 948688.562, 948688.562, 948678.437, 948668.312, 948658.250, 948648.062, 948648.062, 948637.937, 948627.812, 948617.687, 948607.500, 948597.375, 948597.375, 948587.187, 948577.000, 948566.812, 948556.687, 948546.500, 948546.500, 948536.375, 948526.187, 948516.000, 948505.812, 948505.812, 948495.562, 948485.375, 948475.187, 948465.000, 948465.000 };
        public static double[] raw_r2 = new double[] { 904483.125, 904483.250, 904483.250, 904484.000, 904485.687, 904488.562, 904492.695, 904497.687, 904497.687, 904503.875, 904510.937, 904518.750, 904527.312, 904527.312, 904536.437, 904546.125, 904556.187, 904566.562, 904566.562, 904577.312, 904588.312, 904599.500, 904610.812, 904610.812, 904622.375, 904633.937, 904645.687, 904657.500, 904669.312, 904669.312, 904681.250, 904693.187, 904705.250, 904717.312, 904717.312, 904729.375, 904741.375, 904753.437, 904765.500, 904765.500, 904777.625, 904789.687, 904801.812, 904813.875, 904813.875, 904826.000, 904838.062, 904850.187, 904862.312, 904874.500, 904874.500, 904886.625, 904898.750, 904910.812, 904910.812, 904923.000, 904935.125, 904947.250, 904959.312, 904971.375, 904971.375, 904983.500, 904995.562, 905007.750, 905019.875, 905032.000, 905032.000, 905044.125, 905056.187, 905068.312, 905090.375, 905080.375, 905092.500, 905104.500, 905119.625, 905128.687, 905128.687, 905140.750, 905152.875, 905164.937, 905177.000, 905177.000, 905189.062, 905201.125, 905213.125, 905225.250, 905237.250, 905237.250, 905249.312, 905261.375, 905273.500, 905285.500, 905285.500, 905297.562, 905309.625, 905321.687, 905321.687, 905333.750, 905345.812, 905357.875, 905369.937, 905381.937, 905381.937, 905394.000, 905406.062, 905418.125, 905430.125, 905430.125, 905442.250, 905454.312, 905466.375, 905478.375, 905490.375, 905490.375, 905502.375, 905514.375, 905526.437, 905538.437, 905538.437, 905550.500, 905562.562, 905574.562, 905586.500, 905586.500 };
        public static double[] raw_r3 = new double[] { 899448.687, 899448.812, 899448.812, 899449.375, 899450.750, 899453.062, 899456.250, 899456.250, 899460.375, 899465.250, 899470.937, 899477.187, 899484.062, 899484.062, 899491.375, 899499.062, 899507.062, 899515.375, 899515.375, 899524.000, 899532.750, 899541.687, 899550.750, 899550.750, 899559.937, 899569.250, 899578.625, 899588.000, 899597.500, 899597.500, 899607.000, 899616.562, 899626.125, 899635.750, 899635.750, 899645.375, 899654.937, 899664.625, 899674.250, 899683.875, 899683.875, 899693.500, 899703.187, 899712.812, 899722.437, 899722.437, 899732.125, 899741.750, 899751.437, 899761.125, 899761.125, 899770.812, 899780.500, 899790.175, 899790.125, 899799.812, 899809.500, 899819.125, 899828.750, 899838.375, 899838.375, 899848.000, 899857.625, 899867.312, 899877.000, 899877.000, 899886.687, 899896.312, 899905.937, 899915.562, 899925.187, 899925.187, 899934.812, 899944.375, 899954.062, 899963.625, 899963.625, 899973.250, 899982.875, 899992.500, 900002.062, 900002.062, 900011.687, 900021.250, 900030.812, 900040.437, 900050.000, 900050.000, 900059.625, 900069.250, 900078.812, 900088.375, 900088.375, 900097.937, 900107.562, 900117.125, 900117.125, 900126.750, 900136.312, 900145.875, 900155.437, 900155.437, 900165.000, 900174.625, 900184.187, 900193.750, 900203.312, 900203.312, 900212.875, 900222.500, 900232.062, 900241.562, 900241.562, 900241.562, 900260.625, 900270.187, 900279.750, 900289.312, 900289.312, 900298.875, 900308.437, 900317.937, 900327.437, 900327.437 };
        public static double[] raw_r4 = new double[] { 899448.687, 899448.812, 899448.812, 899449.375, 899450.750, 899453.062, 899456.250, 899456.250, 899460.375, 899465.250, 899470.937, 899477.187, 899484.062, 899484.062, 899491.375, 899499.062, 899507.062, 899515.375, 899515.375, 899524.000, 899532.750, 899541.687, 899550.750, 899550.750, 899559.937, 899569.250, 899578.625, 899588.000, 899597.500, 899597.500, 899607.000, 899616.562, 899626.125, 899635.750, 899635.750, 899645.375, 899654.937, 899664.625, 899674.250, 899683.875, 899683.875, 899693.500, 899703.187, 899712.812, 899722.437, 899722.437, 899732.125, 899741.750, 899751.437, 899761.125, 899761.125, 899770.812, 899780.500, 899790.175, 899790.125, 899799.812, 899809.500, 899819.125, 899828.750, 899838.375, 899838.375, 899848.000, 899857.625, 899867.312, 899877.000, 899877.000, 899886.687, 899896.312, 899905.937, 899915.562, 899925.187, 899925.187, 899934.812, 899944.375, 899954.062, 899963.625, 899963.625, 899973.250, 899982.875, 899992.500, 900002.062, 900002.062, 900011.687, 900021.250, 900030.812, 900040.437, 900050.000, 900050.000, 900059.625, 900069.250, 900078.812, 900088.375, 900088.375, 900097.937, 900107.562, 900117.125, 900117.125, 900126.750, 900136.312, 900145.875, 900155.437, 900155.437, 900165.000, 900174.625, 900184.187, 900193.750, 900203.312, 900203.312, 900212.875, 900222.500, 900232.062, 900241.562, 900241.562, 900241.562, 900260.625, 900270.187, 900279.750, 900289.312, 900289.312, 900298.875, 900308.437, 900317.937, 900327.437, 900327.437 };

        public static double[] raw_XX = new double[] { -69, -202, -516, -159, -107, -70, 52, -36, -65, -66, -132, -77, -82, -83, -20, -78, -58, -49, -110, 232, 367, 499 };
        public static double[] raw_YY = new double[] { -1670, -1543, -1612, -1252, -1246, -1474, -1419, -1514, -1481, -1317, -1314, -1231, -1167, -1113, -1017, -990, -960, -982, -995, -768, -759, -817 };
        public static double[] raw_ZZ = new double[] { 7479, 7585, 5515, 6962, 6795, 6561, 7273, 6847, 6845, 6861, 5930, 5905, 5632, 5308, 5670, 5298, 5105, 4899, 3924, 4797, 4680, 4583 };

        public static double[] raw2_XX = new double[] { 110, 111, 65, 88, 65, 57, 52, 24, 25, 8, 7, -10, -20, -31, -40 };
        public static double[] raw2_YY = new double[] { 594, 546, 501, 496, 494, 421, 391, 344, 275, 286, 260, 225, 192, 155, 123 };
        public static double[] raw2_ZZ = new double[] { 4129, 4110, 3559, 3520, 3364, 3155, 3020, 2785, 2563, 2355, 2262, 2053, 1864, 1671, 1514 };

        public static double[] raw_sin = new double[] {0.00, 9.93, 19.47, 28.23, 35.87, 42.07, 46.60, 49.27, 49.98, 48.69, 45.46, 40.42, 33.77, 25.78, 16.75, 7.06, -2.92, -12.78, -22.13, -30.59, -37.84, -43.58, -47.58, -49.68, -49.81, -47.95, -44.17, -38.64, -31.56, -23.23, -13.97, -4.15, 5.83, 15.58, 24.71, 32.85, 39.68, 44.94, 48.40, 49.93, 49.47, 47.04, 42.73, 36.72, 29.25, 20.61, 11.14, 1.24, -8.72, -18.32, -27.20 };

        public FormMenu() {
            InitializeComponent();
            //Simulate_KalmanComparison(raw_XX);
            //Simulate_KalmanSimple(raw_sin);
            var data = new List<double[]>() { new double[] { 0, 0 } };
            double x = 0;
            double y = 0;
            for (int i = 0; i < 100; i++) {
                x += 0.3;
                y += 0.3;
                data.Add(new double[] { x, y });
            }

            int k = 0;
            double[][] sin_2D = raw_sin.Select(r => new double[] { k++, r }).ToArray();
            var sin_noisy = new List<double[]>() { };

            var rand = new Random();
            int noise = 5;
            for (int i = 0; i < raw_sin.Length; i++) {
                double[] row = new double[] { i, sin_2D[i][1] + rand.Next(-noise, noise) };
                sin_noisy.Add(row);
            }

            FormPlot plot = Simulate_UnscentedKalmanFilter(sin_noisy.ToArray());
            AddXYseries(plot, "True [x,y]", sin_2D);
        }

        public void Simulate_ConstantVelocityPrediciton(double[][] raw) {
            //double[][] predicted
            for (int i = 0; i < raw.Length; i++) {

            }
        }

        public FormPlot Simulate_UnscentedKalmanFilter(double[][] raw) {
            int n = 4;
            double dt = 1;
            
            Matrix P = new Matrix(n, n);
            P.SetRow(0, new double[] { 1, 0, 0, 0 });
            P.SetRow(1, new double[] { 0, 1, 0, 0 });
            P.SetRow(2, new double[] { 0, 0, 1, 0 });
            P.SetRow(3, new double[] { 0, 0, 0, 1 });

            Matrix F = new Matrix(4, 4);
            F.SetRow(0, new double[] { 1, dt, 0, 0 });
            F.SetRow(1, new double[] { 0, 1, 0, 0 });
            F.SetRow(2, new double[] { 0, 0, 1, dt });
            F.SetRow(3, new double[] { 0, 0, 0, 1 });

            Matrix Q = new Matrix(4, 4);
            Q.SetRow(0, new double[] { 0.005, 0.01, 0, 0 });
            Q.SetRow(1, new double[] { 0.01, 0.02, 0, 0 });
            Q.SetRow(2, new double[] { 0, 0, 0.005, 0.01 });
            Q.SetRow(3, new double[] { 0, 0, 0.01, 0.02 });
            

            Matrix R = new Matrix(2, 2);
            R.SetRow(0, new double[] { 0.09, 0 });
            R.SetRow(1, new double[] { 0, 0.09 });

            var sigmaPointGenerator = new MerweScaledSigmaPoints(n, 0.1, 2.0, 1.0);
            var physicsModel = new StateTransitionModel(F);
            var measurementSpace = new MeasurementSpace();

            var UKF = new UnscentedKalman(raw[0].ToList(), P, sigmaPointGenerator, physicsModel, measurementSpace, Q, R);
            var UKF_x = new List<double[]>() { };
            for (int i = 1; i < raw.Length; i++) {
                double[] z = raw[i];
                UKF.Predict();
                UKF.Update(z);
                UKF_x.Add(UKF.GetArrayPos());
            }
            
            FormPlot posPlot = PlotSeries("Unscented Kalman - x");
            AddXYseries(posPlot, "Raw [x,y]", raw);
            AddXYseries(posPlot, "UKF [x,y]", UKF_x.Select(r => new double[] { r[0], r[2]}).ToArray());
            return posPlot;
        }

        public void Simulate_KalmanSimple(double[] raw) {
            var x_input = raw;

            var kfPos = new List<double> { };
            var kfVel = new List<double> { };
            var residuals = new List<double> { };
            var deviations = new List<double> { };
            KalmanSimple kf = new KalmanSimple(x_input[0], 0, 1, 0.01, 50, 50, 50);
            foreach (var x in x_input) {
                kf.Predict();
                kf.Update(new double[] { x });
                residuals.Add(kf.GetResidual());
                deviations.Add(kf.GetP());
                kfPos.Add(kf.GetPos());
                kfVel.Add(kf.GetVel());
            }

            FormPlot xPlot = PlotSeries("Kalman Simple - x");
            AddSeries(xPlot, "raw x", x_input.ToList());
            AddSeries(xPlot, "kf x", kfPos.ToList());

            FormPlot resPlot = PlotSeries("Kalman Simple - Residual");
            AddSeries(resPlot, "Residual", residuals);
            AddSeries(resPlot, "Deviations+", deviations);
            AddSeries(resPlot, "Deviations-", deviations.Select(x => 0 - x).ToList());

            FormPlot velPlot = PlotSeries("Kalman Simple - Velocity");
            AddSeries(velPlot, "kf vel", kfVel.ToList());
        }

        public FormPlot PlotSeries(string plotName) {
            FormPlot formPlot = new FormPlot();
            formPlot.Text = plotName;
            formPlot.Show();

            return formPlot;
        }

        public void AddXYseries(FormPlot formPlot, string title, double[][] values) {
            ChartValues<ObservablePoint> xyPoints = new ChartValues<ObservablePoint>();
            for( int i = 0; i < values.Length; i++) {
                xyPoints.Add(new ObservablePoint {
                    X = values[i][0],
                    Y = values[i][1]
                });
            }

            LineSeries lineSeries = new LineSeries {
                Title = title,
                LineSmoothness = 0,
                Values = xyPoints
            };

            formPlot.AddSeries(lineSeries);

        }

        public void AddSeries(FormPlot formPlot, string title, List<double> values) {
            LineSeries lineSeries = new LineSeries {
                Title = title,
                LineSmoothness = 0,
                Values = new ChartValues<double>(values)
            };
            formPlot.AddSeries(lineSeries);
        }

        private void PlotGraphs_Click(object sender, EventArgs e) {

        }


        
        public List<List<double>> SimulateRecieverPositions(double xVel, double yVel, double zVel, double wVel, double maxSpeed, double acceleration, int accelDelay, int noise, int size) {
            var x_simulation = SimulatePosData(xVel, maxSpeed, acceleration, accelDelay, noise, size).ToList();
            var y_simulation = SimulatePosData(yVel, maxSpeed, acceleration, accelDelay, noise, size).ToList();
            var z_simulation = SimulatePosData(zVel, maxSpeed, acceleration, accelDelay, noise, size).ToList();
            var w_simulation = SimulatePosData(wVel, maxSpeed, acceleration, accelDelay, noise, size).ToList();

            return new List<List<double>>() { x_simulation, y_simulation, z_simulation, w_simulation };
        }

        public double[] SimulatePosData(double vel, double max_vel, double accel, int accel_delay, int noise, int size) {
            var pos = 0d;
            var newVel = 0d;
            var data = new List<double>() { pos };
            var random = new Random();

            for (int i = 0; i < size; i++) {
                pos += vel + random.Next(-noise, noise);
                data.Add(pos);
                if (i > accel_delay) {
                    newVel = (vel + accel);
                    if (newVel > max_vel) {
                        vel = max_vel;
                        continue;
                    }

                    if (newVel <= -max_vel) {
                        vel = -max_vel;
                        continue;
                    }

                    vel = newVel;
                }
            }

            return data.ToArray();
        }


        public void Simulate_Isan2Mono(List<double> raw1, List<double> raw2, List<double> raw3, List<double> raw4) {
            int maxSize = 0;
            double time = 0.6;
            raw1.RemoveRange(raw_r1.Length - maxSize, maxSize);
            raw2.RemoveRange(raw_r2.Length - maxSize, maxSize);
            raw3.RemoveRange(raw_r3.Length - maxSize, maxSize);
            raw4.RemoveRange(raw_r4.Length - maxSize, maxSize);

            Isan2Mono isanSim = new Isan2Mono();
            isanSim.UpdateStaggered(raw1.ToArray(), raw2.ToArray(), raw3.ToArray(), raw4.ToArray());

            var rawVel = GetVelocities(time, isanSim.GetX().ToArray(), isanSim.GetY().ToArray(), isanSim.GetZ().ToArray());
            FormPlot velPlot = PlotSeries("Isan2Mono - Velocity");
            AddSeries(velPlot, "raw vel", rawVel);

            FormPlot xPlot = PlotSeries("Isan2Mono - X");
            AddSeries(xPlot,"raw x", isanSim.GetX().ToList());

            FormPlot yPlot = PlotSeries("Isan2Mono - Y");
            AddSeries(yPlot, "raw y", isanSim.GetY().ToList());

            FormPlot zPlot = PlotSeries("Isan2Mono - Z");
            AddSeries(zPlot, "raw z", isanSim.GetZ().ToList());
        }

        public void Simulate_IsanKalman(List<double> raw1, List<double> raw2, List<double> raw3, List<double> raw4) {
            int maxSize = 0;
            double time = 0.6;
            raw1.RemoveRange(raw_r1.Length - maxSize, maxSize);
            raw2.RemoveRange(raw_r2.Length - maxSize, maxSize);
            raw3.RemoveRange(raw_r3.Length - maxSize, maxSize);
            raw4.RemoveRange(raw_r4.Length - maxSize, maxSize);

            IsanKalmanV2 isanKF = new IsanKalmanV2();
            isanKF.UpdateStaggered(raw1.ToArray(), raw2.ToArray(), raw3.ToArray(), raw4.ToArray());

            var rawVel = GetVelocities(time, isanKF.GetX().ToArray(), isanKF.GetY().ToArray(), isanKF.GetZ().ToArray());

            var kalVel = new List<double>();
            for (int i = 0; i < isanKF.GetVelX().Count; i++) {
                kalVel.Add(
                    Sqrt(Pow(isanKF.GetVelX()[i], 2) + Pow(isanKF.GetVelY()[i], 2) + Pow(isanKF.GetVelZ()[i], 2))
                );
            }

            FormPlot velPlot = PlotSeries("Isan2Mono - Velocity");
            AddSeries(velPlot, "raw vel", rawVel);
            AddSeries(velPlot, "kf vel", kalVel);

            FormPlot xPlot = PlotSeries("Isan2Mono - X");
            AddSeries(xPlot, "raw x", isanKF.GetX().ToList());

            FormPlot yPlot = PlotSeries("Isan2Mono - Y");
            AddSeries(yPlot, "raw y", isanKF.GetY().ToList());

            FormPlot zPlot = PlotSeries("Isan2Mono - Z");
            AddSeries(zPlot, "raw z", isanKF.GetZ().ToList());

        }


        public List<double> GetVelocities(double time, double[] x, double[] y, double[] z) {
            List<double> vel = new List<double>();
            for (int i = 1; i < x.Length; i++) {
                double velX = (x[i] - x[i - 1]);
                double velY = (y[i] - y[i - 1]);
                double velZ = (z[i] - z[i - 1]);
                vel.Add(Sqrt((velX * velX) + (velY * velY) + (velZ * velZ)) / time);
            }
            return vel;
        }

        public void Simulate_KalmanComparison(double[] raw_x) {

            //Plot Raw Data
            FormPlot posPlot = PlotSeries("Kalman Comparison - Pos");
            AddSeries(posPlot, "raw x", raw_x.ToList());

            double calculationTick = 1.2;
            var raw_v = GetSpeed(raw_x.ToList(), calculationTick);
            FormPlot velPlot = PlotSeries("Kalman Comparison - Vel");
            AddSeries(velPlot, "raw vel", raw_v);

            //Input Data in different Kalman Filters
            double accel = 0.01;
            double obv_variance = 0.1;
            var kalmanDict = new Dictionary<string, IKalman>();
            kalmanDict.Add("Kalman", new Kalman(raw_x[0], raw_v[0], calculationTick, accel, obv_variance));
            kalmanDict.Add("Bowtie", new KalmanBowtie(raw_x[0], raw_v[0], accel, calculationTick, 0.1, 0.1, obv_variance, obv_variance));
            kalmanDict.Add("Simple", new KalmanSimple(raw_x[0], raw_v[0], calculationTick, accel, 0.1, 0.1, obv_variance));
            kalmanDict.Add("AlphaBeta", new AlphaBetaFilter(raw_x[0], calculationTick, 1, 1));

            foreach (var kalman in kalmanDict) {
                var vel = new List<double>();
                var pos = new List<double>();
                for (int i = 0; i < raw_x.Length; i++) {
                    kalman.Value.Predict();

                    double[] z = new double[] { raw_x[i], raw_v[i] };
                    kalman.Value.Update(z);
                    pos.Add(kalman.Value.GetPos());
                    vel.Add(kalman.Value.GetVel());
                }
                AddSeries(posPlot, kalman.Key + "X", pos);
                AddSeries(velPlot, kalman.Key + "Y", vel);
            }
        }

        public List<double> GetDisplacement(double[] raw_x, double[] raw_y, double[] raw_z) {
            var displacement = new List<double>();
            double x = raw_x[0];
            double y = raw_y[0];
            double z = raw_z[0];
            for (int i = 0; i < raw_x.Length; i++) {
                displacement.Add(
                    Sqrt(Pow(raw_x[i] - x, 2) + Pow(raw_y[i] - y, 2) + Pow(raw_z[i] - z, 2))
                );
                x = raw_x[i]; y = raw_y[i]; z = raw_z[i];
            }

            return displacement;
        }

        public List<double> GetSpeed(List<double> positions, double timeStep) {
            List<double> vel = new List<double>();
            double lastPos = positions[0];//y
            foreach (double pos in positions) {
                vel.Add((pos - lastPos) / timeStep);
                lastPos = pos;
            }
            return vel;
        }

        private void textBox1_TextChanged(object sender, EventArgs e) { }

        private void button1_Click(object sender, EventArgs e) { }

        private void Form1_Load(object sender, EventArgs e) { }

    }

    public class Simulator {
        double timeStep;
        double sensorTime;
        double calculateTime;

        public Simulator() { }
        
        public Simulator(double timeStep, double sensorTime, double calculateTime) {
            this.timeStep = timeStep;
            this.sensorTime = sensorTime;
            this.calculateTime = calculateTime;
        }

        public double[][] GetCalculatorHits(double[][] data, int precision) {
            var dataNew = new List<List<double>>() { };

            for (int d = 0; d < data.Length; d++ ) {
                dataNew.Add(new List<double> { });
            }

            int i = 0;
            double iSensor = Math.Round(sensorTime, precision);
            double iCalculate = Math.Round(calculateTime, precision);
            int elementLength = data.First().Length;
            double sensorTimeline = iSensor;
            double calculateTimeline = iCalculate;

            for ( double t = 0;   t < (elementLength*sensorTime);   t = Math.Round(t+timeStep, precision) ) {
                if (t >= sensorTimeline) {
                    sensorTimeline += iSensor;
                    i++;
                }

                if (t >= calculateTimeline) {
                    calculateTimeline += iCalculate;
                    for (int j = 0; j < data.Length; j++) {
                        dataNew.ElementAt(j).Add(data[j][i]);
                    }
                }
            }

            double[][] dataArray = new double[data.Length][];
            for (int d = 0; d < data.Length; d++) {
                dataArray[d] = dataNew[d].ToArray();
            }

            return dataArray;
        }

        public double[][] GetSensorInput(double[][] data, int precision) {
            var dataNew = new List<List<double>>() { };

            for (int d = 0; d < data.Length; d++) {
                dataNew.Add(new List<double> { });
            }

            int i = 0;
            double iSensor = Math.Round(sensorTime, precision);
            double iCalculate = Math.Round(calculateTime, precision);
            int elementLength = data.First().Length;
            double sensorTimeline = iSensor;

            for (double t = 0; t < (elementLength * sensorTime); t = Math.Round(t + timeStep, precision)) {
                if (t >= sensorTimeline) {
                    sensorTimeline += iSensor;
                    i++;
                }

                for (int j = 0; j < data.Length; j++) {
                    dataNew.ElementAt(j).Add(data[j][i]);
                }
            }

            double[][] dataArray = new double[data.Length][];
            for (int d = 0; d < data.Length; d++) {
                dataArray[d] = dataNew[d].ToArray();
            }

            return dataArray;
        }
    }


    public class KalmanContext
    {
        private IKalman _kalman;
        public KalmanContext() { }
        public KalmanContext(IKalman kalman) {
            this._kalman = kalman;
        }

        public void SetStrategy(IKalman strategy) {
            this._kalman = strategy;
        }

        public List<double[]> FilterData() {
            throw new NotImplementedException();
        }

    }
}
