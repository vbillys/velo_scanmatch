#ifndef HELPER_YAW_ANGLE_CALC
#define HELPER_YAW_ANGLE_CALC
static double NORMALIZE(double z)
{
  return atan2(sin(z),cos(z));
}

static double ANGLEDIFF (double a, double b)
{
  double d1, d2;
  a = NORMALIZE(a);
  b = NORMALIZE(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}
#endif

