public class Matrix
{
    double[][] matrix;

    public Matrix() { }
    /*
     [[1,0],
      [0,1]]
    */

    public Matrix(int rows, int cols) {
        double[][] result = new double[rows][];
        for (int i = 0; i < rows; ++i)
            result[i] = new double[cols];
        matrix = result;
    }

    public void Set(int row, int col, double val) {
        matrix[row][col] = val;
    }

    public void SetRow(int row, double[] vals) {
        for (int i = 0; i < vals.Length; i++) {
            matrix[row][i] = vals[i];
        }
    }

    public double Get(int row, int col) {
        return matrix[row][col];
    }

    public double[] GetColumn(int col) {
        double[] column = new double[matrix.Length];
        for (int i = 0; i < matrix.Length; i++) {
            column[i] = matrix[i][col];
        }
        return column;
    }

    public double[] GetRow(int row) {
        return matrix[row];
    }

    public double[][] GetDouble() {
        return matrix;
    }


    public Matrix Multiply(Matrix m) {
        double dotProduct = 0;
        int rowLen = m.GetColumn(0).Length;
        int colLen = matrix[0].Length;
        if (rowLen != colLen) {
            throw new Exception("Matrix column size is not equal to multiplying matrices row size");
        };

        Matrix product = new Matrix(matrix.Length, m.GetRow(0).Length);
        //A -> 0,0 0,1 0,2
        //B -> 0,0 1,0 2,0
        for (int i = 0; i < matrix.Length; i++) {
            double[] row = matrix[i];

            for (int j = 0; j < m.GetRow(0).Length; j++) {
                for (int k = 0; k < row.Length; k++) {
                    double a = matrix[i][k];
                    double b = m.Get(k, j);
                    dotProduct += a * b;
                }
                product.Set(i, j, dotProduct);
                dotProduct = 0;
            }

        }

        return product;
    }

    public Matrix Multiply(double s) {
        Matrix product = new Matrix(matrix.Length, matrix[0].Length);
        for (int i = 0; i < matrix.Length; i++) {
            for (int j = 0; j < matrix[0].Length; j++) {
                product.Set(i, j, matrix[i][j] * s);
            }
        }
        return product;
    }

    public Matrix Transpose() {
        Matrix Transpose = new Matrix(matrix[0].Length, matrix.Length);
        for (int i = 0; i < matrix.Length; i++) {
            for (int j = 0; j < matrix[0].Length; j++) {
                Transpose.Set(j, i, matrix[i][j]);
            }
        }
        return Transpose;
    }

    public Matrix Add(Matrix m) {
        Matrix A = new Matrix(matrix.Length, matrix[0].Length);
        int rowLen = m.GetRow(0).Length;
        int colLen = m.GetColumn(0).Length;
        if (colLen == matrix.Length &&
            rowLen == matrix[0].Length) {
            for (int i = 0; i < matrix.Length; i++) {
                for (int j = 0; j < matrix[0].Length; j++) {
                    A.Set(i, j, m.Get(i, j) + matrix[i][j]);
                }
            }
        } else {
            throw new Exception("Cannot add Matrices as they're not the same dimensions");
        }
        return A;
    }

    public Matrix Subtract(Matrix m) {
        Matrix A = new Matrix(matrix.Length, matrix[0].Length);
        int rowLen = m.GetRow(0).Length;
        int colLen = m.GetColumn(0).Length;
        if (colLen == matrix.Length &&
            rowLen == matrix[0].Length) {
            for (int i = 0; i < matrix.Length; i++) {
                for (int j = 0; j < matrix[0].Length; j++) {
                    A.Set(i, j, matrix[i][j] - m.Get(i, j));
                }
            }
        } else {
            throw new Exception("Cannot subtract Matrices as they're not the same dimensions");
        }
        return A;
    }

    public static Matrix Invert_2x2(double[][] mat) {
        if (mat.Length != 2 && mat[0].Length != 2) {
            if (mat.Length != 1 && mat[0].Length != 1) {
                throw new Exception("Matrix must be 2x2 or 1x1");
            }
        }

        if (Determinant(mat) == 0) {
            throw new Exception("Matrix determinant is zero and so cannot be inverted");
        }

        //2x2 Inverse
        if (mat.Length == 2 && mat[0].Length == 2) {
            Matrix inverse = new Matrix(2, 2);
            double a = mat[0][0];
            double b = mat[0][1];
            double c = mat[1][0];
            double d = mat[1][1];

            double denominator = (a * d) - (b * c);
            inverse.Set(0, 0, d / denominator);
            inverse.Set(0, 1, -b / denominator);
            inverse.Set(1, 0, -c / denominator);
            inverse.Set(1, 1, a / denominator);

            return inverse;
            //1x1 Inverse
        } else {
            Matrix inverse = new Matrix(1, 1);
            inverse.Set(0, 0, 1 / mat[0][0]);
            return inverse;
        }


    }

    public static double Determinant(double[][] mat) {
        if (mat.Length != mat[0].Length) { throw new Exception("Matrix is not sqaure"); };
        double result = 0;
        if (mat.Length == 1) {
            result = mat[0][0];
            return result;
        }

        if (mat.Length == 2) {
            result = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
            return result;
        }

        for (int i = 0; i < mat[0].Length; i++) {
            double[][] temp = new double[mat.Length - 1][];
            for (int j = 0; j < temp.Length; j++) {
                temp[j] = new double[temp.Length];
            }

            for (int k = 1; k < mat.Length; k++) {
                Array.Copy(mat[k], 0, temp[k - 1], 0, i);
                Array.Copy(mat[k], i + 1, temp[k - 1], i, mat[0].Length - i - 1);
            }
            result += mat[0][i] * Math.Pow(-1, i) * Determinant(temp);

        }
        return result;
    }
}