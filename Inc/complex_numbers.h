#ifndef __COMPLEX_NUMBERS__H
#define __COMPLEX_NUMBERS__H

typedef struct 
{
    float Re;
    float Im;    
}ComplexNumber;

static float squareRoot(float x);

ComplexNumber CplxSum(ComplexNumber x, ComplexNumber y);
ComplexNumber CplxDif(ComplexNumber x, ComplexNumber y);
ComplexNumber CplxMul(ComplexNumber x, ComplexNumber y);
ComplexNumber CplxDiv(ComplexNumber x, ComplexNumber y);
float CplxMag(ComplexNumber x);

#endif
