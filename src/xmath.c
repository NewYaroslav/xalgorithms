#include "xmath.h"
#include "math.h"

#define	ROUND_ERROR	1e-10
#define	POW3(x)	((x) * (x) * (x))
#define	POW2(x)	((x) * (x))

unsigned int xa_quadratic_solve(double a, double b, double c, double x[2]) {
	double tmp;
	// На самом деле просто линейное уравнение
	if (a == 0) {
		if (b == 0)
			return (0);
		x[0] = -c / b;
		return (1);
	}

	tmp = POW2(b) - 4 * a * c;
	if (tmp > ROUND_ERROR) {
		double tmp_sqrt = sqrt(tmp);
		x[0] = (-b + tmp_sqrt) / (2 * a);
		x[1] = (-b - tmp_sqrt) / (2 * a);
		return (2);
	} else if (tmp > -ROUND_ERROR) {
		x[0] = -b / (2 * a);
		return (1);
	} else {
		return (0);
	}
}

double xa_linear_interpolation(double x, double x1, double y1, double x2, double y2) {
	return (((x - x1) / (x2 - x1)) * (y2 - y1) + y1);
}
