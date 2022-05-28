#include "complex_numbers.h"
#include <math.h>

ComplexNumber CplxSum(ComplexNumber x, ComplexNumber y)
{
   ComplexNumber result; 
   result.Re = x.Re + y.Re;    
   result.Im = x.Im + y.Im;       
   return  result;          
}
              
ComplexNumber CplxDif(ComplexNumber x, ComplexNumber y)
{
   ComplexNumber result; 
   result.Re = x.Re - y.Re;    
   result.Im = x.Im - y.Im;          
   return  result;             
}
             
ComplexNumber CplxMul(ComplexNumber x, ComplexNumber y)
{
   ComplexNumber result; 
   result.Re = x.Re*y.Re - x.Im*y.Im;    
   result.Im = x.Im*y.Re + x.Re*y.Im;            
   return  result;           
}
              
ComplexNumber CplxDiv(ComplexNumber x, ComplexNumber y)
{
   ComplexNumber result;
   float norm = y.Re*y.Re + y.Im*y.Im;
   
   result.Re = (x.Re*y.Re + x.Im*y.Im)/norm;    
   result.Im = (x.Im*y.Re - x.Re*y.Im)/norm;        
   return  result;           
}

float CplxMag(ComplexNumber x)
{
	return squareRoot(x.Re*x.Re + x.Im*x.Im);
}

static float squareRoot(float x) 
{
	float guess = 1;
	int lim = 100;

	while( (fabs(guess*guess - x) >= 0.000002 )&&(lim-- >0))
		guess = ((x/guess) + guess) * 0.5;

	return (guess);
}
