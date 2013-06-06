#include "drive_train.h"

// linear mapping classs implementation

mapLinearMoves::mapLinearMoves()
{
   aMinVal = 0;
   aMaxVal = 0;
   slope = 0;
   intercept = 0;
   aMidVal = 0;
   iMinVal = 0;
   iMaxVal = 0;
   iMidVal = 0;
}

void mapLinearMoves::initMap(double actualMinVal, double actualMaxVal, double inpMinVal, double inpMaxVal)
{
    //compute the intercept and slope
    slope = (actualMaxVal - actualMinVal) / (inpMaxVal - inpMinVal);
    intercept = actualMaxVal - (slope*inpMaxVal);
     
    //set the values
    aMinVal = actualMinVal;
    aMaxVal = actualMaxVal;
    iMinVal = inpMinVal;
    iMaxVal = inpMaxVal;
    iMidVal = (inpMaxVal + inpMinVal)/2;
    aMidVal = intercept; 
}

void mapLinearMoves::initMap(double actualMinVal, double actualMaxVal, 
                               double inpMinVal, double inpMaxVal, 
                               double inpMidVal)
{
    //compute the intercept and slope
    slope = (actualMaxVal - actualMinVal) / (inpMaxVal - inpMinVal);
    intercept = actualMaxVal - (slope*inpMaxVal);
     
    //set the values
    aMinVal = actualMinVal;
    aMaxVal = actualMaxVal;
    iMinVal = inpMinVal;
    iMaxVal = inpMaxVal;
    iMidVal = (inpMaxVal + inpMinVal)/2;
    aMidVal = inpMidVal; 
}

double mapLinearMoves::getValue(double inpVal)
{
     //create the value
     double currValue = aMinVal;

     //compute the current value
     currValue = slope*inpVal + intercept;

     //check the bounds
     if (currValue < aMinVal)
     {
         //set it to the minium
         currValue = aMinVal;
     }

     if (currValue > aMaxVal)
     {
         //set it to the maximum
         currValue = aMaxVal;
     }

     //check to see if it is in the middle
     if (inpVal == iMidVal)
     {
         currValue = aMidVal;
     }

     //return the value
     return(currValue);
}


