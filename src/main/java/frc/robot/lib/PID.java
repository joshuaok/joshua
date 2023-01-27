package frc.robot.lib;

public class PID {

  
  public double proportionalGain = 0;
  public double integralGain = 0;
  public double derivativeGain = 0;
  public double integral_charge = 0;


  public double upperCageLimit = 0;
  public double lowerCageLimit = 0;

  public double upperDeadbandLimit = 0;
  public double lowerDeadbandLimit = 0;

  public double maxOutput = 0;
  public double minOutput = 0;

  public double output = 0;
  public double error = 0;

  public double derivativeCalculation = 0;

  public double previousError = 0;

  public double deltaTime = 0.02;

  public double cmd = 0;
  public double feed = 0;

  public boolean deadband_active = false;

  public boolean enableCage = true;
  public boolean enableDeadBand = true;

  public static int deadband_counter = 0;

  public void initialize(double _P,double _I, double _D,
   double Cage_Limit,double Deadband,double MaxOutput) {
    proportionalGain = _P;
    integralGain = _I;
    derivativeGain = _D;
    
    upperCageLimit = +Cage_Limit;
    lowerCageLimit = -Cage_Limit;
  
    upperDeadbandLimit = +Deadband;
    lowerDeadbandLimit = -Deadband;
  
    maxOutput = +MaxOutput;
    minOutput = -MaxOutput;
  
    output = 0;
    error = 0;
  
    derivativeCalculation = 0;
    integral_charge = 0;
    previousError = 0;
  
    deltaTime = 0.02;

    deadband_active = false; 

    enableCage = true;
    enableDeadBand = true;
    
  }

  public void initialize2(double _P,double _I, double _D,
     double Cage_Limit,double Deadband,double MaxOutput, boolean enable_Cage, boolean enable_DeadBand) { 
    initialize(_P, _I, _D, Cage_Limit, Deadband, MaxOutput);

    enableCage = enable_Cage;
    enableDeadBand = enable_DeadBand;


  }

  public double execute(double command, double feedback) {
    cmd = command;
    feed = feedback;

    error = command - feedback;

    derivativeCalculation = (error - previousError) / deltaTime;
    
    previousError = error; 

    if (enableDeadBand == true) {
      if((error < upperDeadbandLimit) && (error > lowerDeadbandLimit)) {
        deadband_counter++;
        if(deadband_counter > 13) { // 13 = 50 / 4 = 50 ms split into 4 quarters
          deadband_active = true;
          deadband_counter = 13;
        }
          
      }
    }
    
    if (enableCage == true) {
      if((error < upperCageLimit) && (error > lowerCageLimit)){
        deadband_counter = 0;
        integral_charge = integral_charge + error*deltaTime;
        deadband_active = false; 
      }
    }

    else {
      integral_charge = integral_charge + error*deltaTime;
    }


    output = error*proportionalGain + integral_charge*integralGain - derivativeCalculation*derivativeGain;
    if(output > maxOutput) {
      output = maxOutput;
    } else if(output < minOutput) {
      output = minOutput;
    }
    return output;
  }

  public void reset() {
    output = 0;
    error = 0;
  
    derivativeCalculation = 0;
    integral_charge = 0;
    previousError = 0;

    deadband_active = false;
  }
}