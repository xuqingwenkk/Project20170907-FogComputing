//
// Created by xuqw on 17-8-12.
//

#ifndef OMNISLAM_MISC_H
#define OMNISLAM_MISC_H

namespace OmniSLAM{

#ifndef M_PID
#define M_PID   3.1415926535897932384626433832795028841971693993
#endif

#ifndef M_PIf
#define M_PIf    3.1415926535897932384626f
#endif

    const double RHOd = 180.0 / M_PID;
    const float  RHOf = 180.0f / M_PIf;

    /**
	* evaluate a polynom using Horner
	*
	* @param coeffs  T* of coefficients
	* @param s		 number of polynomial coefficients
	* @param x		 x value to evaluate
	*
	* @return      function value
	*/
    inline double horner(
            const double* coeffs, const int& s, const double& x)
    {
        double res = 0.0;
        for (int i = s - 1; i >= 0; i--)
            res = res * x + coeffs[i];
        return res;
    }
}

#endif //PROJECT_MISC_H
