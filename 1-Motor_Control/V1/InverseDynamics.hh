
class InverseDynamics{
  private:
  public:
  InverseDynamics() = default;
  float Torque(float angle, float velocity, float mass,float sampleTime){
    mass2 = mass;
    prevVelocity = velocity;
    torque3 = torque2;
    torque2 = torque1;
    torque1 = (Je+mass*sq(lp))*acceleration+0.5*Me*le*g*cos(angle);
    return torque1;
  } 
    float lp = 0.3;
    float le = 0.17;
    float Me = 0.1;
    float Je = 0.21;//0.2 + mass2*sq(lp)+Me*sq(le);
    float mass2 = 0;
    float g = 9.81;
    float prevVelocity = 0.f;
    float acceleration = 0.f;
    float torque1 = 0.f;
    float torque2 = 0.f;
    float torque3 = 0.f;
    float prevAcceleration = 0.f; 
};
