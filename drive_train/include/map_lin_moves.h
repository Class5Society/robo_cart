//create mapping class
class mapLinearMoves
{
public:
  mapLinearMoves();
  void initMap(double actualMinVal, double actualMaxVal, double inpMinVal, double inpMaxVal);
  void initMap(double actualMinVal, double actualMaxVal, double inpMinVal, double inpMaxVal, double inpMidVal);
  double getValue(double inpVal);
private:
   double aMinVal;
   double aMaxVal;
   double slope;
   double intercept;
   double aMidVal;
   double iMinVal;
   double iMaxVal;
   double iMidVal;
};

