#ifndef CSCI7551_PROJECT_GRAPH_COSTFUNCTION_H_
#define CSCI7551_PROJECT_GRAPH_COSTFUNCTION_H_

#include <cmath>

namespace csci7551_project
{

  enum COST_FUNCTION { NONE, SMOCK, OVERGAARD, BPL };

  class CostFunction
  {
  public:
    virtual double cost(double flow, double travelTime, double capacity) = 0;
    // virtual ~CostFunction() = 0;
  };

  class Smock : public CostFunction
  {
  public:
    double cost(double flow, double travelTime, double capacity)
    {
      return travelTime * exp(flow / capacity);
    }
    // ~Smock() {}
  };

  class Overgaard : public CostFunction
  {
  public:
    Overgaard(double a, double b):
      alpha(a),
      beta(b) {}
    double cost(double flow, double travelTime, double capacity)
    {
      double exponent = beta * (flow / capacity);
      return travelTime * pow(alpha, exponent);
    }
  private:
    double alpha, beta;
  };

  class BureauOfPublicLands : public CostFunction
  {
  public:
    BureauOfPublicLands(double a, double b):
      alpha(a),
      beta(b) {}
    double cost(double flow, double travelTime, double capacity)
    {
      double quantity = flow / capacity;
      double rightTerm = alpha * pow(quantity, beta);
      return travelTime + travelTime * rightTerm;
    }
  private:
    double alpha, beta;
  };

  class CostFunctionFactory
  {
  public:
    CostFunctionFactory()
    {
      alpha = 0;
      beta = 0;
    }
    CostFunctionFactory(double a, double b):
      alpha(a),
      beta(b) {}
    inline CostFunction* operator()(COST_FUNCTION c)
    {
      if (c == SMOCK)
        return new Smock();
      else if (c == OVERGAARD)
        return new Overgaard(alpha, beta);
      else if (c == BPL)
        return new BureauOfPublicLands(alpha, beta);
      else
        return 0;
    }
  private:
    double alpha, beta;
  };
}

#endif