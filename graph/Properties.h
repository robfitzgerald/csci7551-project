namespace csci7551_project
{
  class Property 
  {
  public:
    virtual double cost() = 0;
    virtual double cost(double) = 0;
    // virtual double weight() = 0;
  };

  class TestProperty : public Property
  {
  public:
    TestProperty(double w): weight(w) {}
    double cost() { return weight; }
    double cost(double V) { return weight * V; }
  private:
    double weight;
  };

  class P_Smock : public Property
  {
  public:
    P_Smock(double f, double s): freeFlowTravelTime(f), steadyStateCapacity(s) 
    { 
      flowVPH = 0; 
    }
    void setV(double V) { flowVPH = V; }
    double cost()
    {
      return freeFlowTravelTime * exp(flowVPH / steadyStateCapacity);
    }

    double cost(double V)
    {
      return freeFlowTravelTime * exp(V / steadyStateCapacity);
    }
  private:
    double freeFlowTravelTime, steadyStateCapacity, flowVPH;
  };  
}