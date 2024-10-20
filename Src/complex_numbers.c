#include "complex_numbers.h"
#include <math.h>

/**
  * @brief  Calculate sum of two complex numbers
  * @param  x, y - complex numbers to sum
  * @retval sum of x and y
  */
ComplexNumber CplxSum(ComplexNumber x, ComplexNumber y)
{
   ComplexNumber result; 
   result.Re = x.Re + y.Re;    
   result.Im = x.Im + y.Im;       
   return  result;          
}

/**
  * @brief  Calculate difference of two complex numbers
  * @param  x, y - complex numbers to difference
  * @retval difference of x and y
  */
ComplexNumber CplxDif(ComplexNumber x, ComplexNumber y)
{
   ComplexNumber result; 
   result.Re = x.Re - y.Re;    
   result.Im = x.Im - y.Im;          
   return  result;             
}

/**
  * @brief  Calculate product of two complex numbers
  * @param  x, y - complex numbers to product
  * @retval product of x and y
  */
ComplexNumber CplxMul(ComplexNumber x, ComplexNumber y)
{
   ComplexNumber result; 
   result.Re = x.Re*y.Re - x.Im*y.Im;    
   result.Im = x.Im*y.Re + x.Re*y.Im;            
   return  result;           
}

/**
  * @brief  Calculate division of two complex numbers
  * @param  x, y - complex numbers to divide
  * @retval division of x and y
  */
ComplexNumber CplxDiv(ComplexNumber x, ComplexNumber y)
{
   ComplexNumber result;
   float norm = y.Re*y.Re + y.Im*y.Im;
   
   result.Re = (x.Re*y.Re + x.Im*y.Im)/norm;    
   result.Im = (x.Im*y.Re - x.Re*y.Im)/norm;        
   return  result;           
}

/**
  * @brief  Calculate magnitude of complex number
  * @param  x - complex number
  * @retval magnitude of x
  */
float CplxMag(ComplexNumber x)
{
	return squareRoot(x.Re*x.Re + x.Im*x.Im);
}

/**
  * @brief  Calculate square root of number
  * @param  x - number
  * @retval square root of x
  */
static float squareRoot(float x) 
{
	float guess = 1;
	int lim = 100;

	while( (fabs(guess*guess - x) >= 0.000002 )&&(lim-- >0))
		guess = ((x/guess) + guess) * 0.5;

	return (guess);
}
