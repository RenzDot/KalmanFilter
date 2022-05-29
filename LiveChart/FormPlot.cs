using LiveCharts;
using LiveCharts.Wpf;
using System.Windows.Media;
using System.Windows.Forms;

namespace Hamilton
{
    public partial class FormPlot : Form
    {

        public FormPlot() {
            InitializeComponent();
            cartesianChart1.Zoom = ZoomingOptions.Xy;
            cartesianChart1.LegendLocation = LegendLocation.Right;

            Separator sepX = new Separator() {
                IsEnabled = true,
                StrokeThickness = 0.1,
                Stroke = Brushes.Black
            };

            Separator sepY = new Separator() {
                IsEnabled = true,
                StrokeThickness = 0.1,
                Stroke = Brushes.Black
            };

            cartesianChart1.AxisX.Add( new Axis() { Separator = sepX } );
            cartesianChart1.AxisY.Add( new Axis() { Separator = sepY } );
        }

        public void AddSeries(LineSeries chart) {
            cartesianChart1.Series.Add(chart);
        }

    }
}
