#include <eloquent_tensorflow32.h> 

// LSTM Pre-trained model// LSTM Pre-trained model
#include "model.h"

// Model parameters
constexpr uint16_t NUM_OPS {8};
constexpr uint16_t ARENA_SIZE {22136*2};

using Eloquent::Esp32::TensorFlow;

class InverseDynamics{
  private:
    TensorFlow<NUM_OPS, ARENA_SIZE> tf;
  public:
  InverseDynamics() = default;
  float Torque(float angle, float velocity, float mass,float sampleTime){
    mass2 = mass;
    prevVelocity = velocity;
    torque3 = torque2;
    torque2 = torque1;
    torque1 = (Je+mass*sq(lp))*acceleration;//+0*Me*le*g*sin(angle)+mass*lp*sin(angle)*g;
    return torque1;
  }
  float diffTorque(float torque){
    diffTorque3 = diffTorque2;
    diffTorque2 = diffTorque1;
    diffTorque1 = torque - torque2;
    return diffTorque1;
  }
  float invDiffTorque(float ScaledDiffTorque){
    // //Using estimate of torque1
    float torque1_estimate=scaledDiffTorque2+torque2;
    scaledDiffTorque1 = ScaledDiffTorque + torque1_estimate;
    // //Using true value of torque1 (which is the worse option)
    // scaledDiffTorque1 = ScaledDiffTorque + torque1;
    scaledDiffTorque2 = ScaledDiffTorque;
    
    /*Serial.print("Debug");
    Serial.print(ScaledDiffTorque,5);
    Serial.print(",");
    Serial.print(torque2,5);
    Serial.println("------");*/
    return scaledDiffTorque1;
  }
  void LSTMsetup(){
      // replace with the correct values
    tf.setNumInputs(3);
    tf.setNumOutputs(1);
    // add required ops

    tf.resolver.AddReshape();
    tf.resolver.AddFullyConnected();
    tf.resolver.AddUnpack();
    tf.resolver.AddSplit();
    tf.resolver.AddLogistic();
    tf.resolver.AddMul();
    tf.resolver.AddAdd();
    tf.resolver.AddTanh();

    while (!tf.begin(g_model).isOk()) 
        Serial.println(tf.exception.toString());
  }
  float LSTMpredict(float torque1scaled){
    float input[3] = {torque3scaled,torque2scaled,torque1scaled};
    
    while (!tf.predict(input).isOk())
        Serial.println(tf.exception.toString());
    
    tf.predict(input);
    torque3scaled = torque2scaled;
    torque2scaled = torque1scaled;
    return tf.result();
  }
  float Scaling_transformation(float X, float xmin, float xmax,float min_X,float max_X){
    float X_std = (X - min_X) / (max_X - min_X);
    float X_scaled = X_std * (xmax - xmin) + xmin;
    return X_scaled;
  }
  float Inverse_Scaling_transformation(float X_scaled, float min_val, float max_val, float MIN, float MAX){
    float X_std = (X_scaled - min_val) / (max_val - min_val);
    float X_original = X_std * (MAX - MIN) + MIN;
    return X_original;
}
    float lp = 0.3;
    float le = 0.07;
    float Me = 0.1;
    float Je = 0.21;//0.2 + mass2*sq(lp)+Me*sq(le);
    float mass2 = 0;
    float g = 9.81;
    float prevVelocity = 0.f;
    float acceleration = 0.f;
    float torque1 = 0.f;
    float torque2 = 0.f;
    float torque3 = 0.f;
    float torque2scaled = 0.f;
    float torque3scaled = 0.f;
    float diffTorque1 = 0.f;
    float diffTorque2 = 0.f;
    float diffTorque3 = 0.f;
    float scaledDiffTorque2 = 0.f;
    float scaledDiffTorque1 = 0.f;
    float prevAcceleration = 0.f; 
};
