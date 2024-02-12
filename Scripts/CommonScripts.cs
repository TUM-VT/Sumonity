using UnityEngine;


public class PIController
{
    private float kp; // Proportional gain
    private float ki; // Integral gain
    
    private float integralError = 0f; // Accumulated error

    public PIController(float kp, float ki)
    {
        this.kp = kp;
        this.ki = ki;
    }

    public float Control(float setpoint, float actualVelocity)
    {
        // Calculate the error between desired setpoint and actual velocity
        float error = setpoint - actualVelocity;

        // Proportional term
        float proportionalTerm = kp * error;

        // Integral term
        integralError += error * Time.fixedDeltaTime;
        float integralTerm = ki * integralError;

        // Calculate the torque output using the PI controller
        float torque = proportionalTerm + integralTerm;

        return torque;
    }

    // This is useful if you want to reset the accumulated error under certain conditions
    public void ResetIntegral()
    {
        integralError = 0f;
    }
}



public class PIDController
{
    private float kp; // Proportional gain
    private float ki; // Integral gain
    private float kd; // Derivative gain
    
    private float integralError = 0f; // Accumulated error
    private float previousError = 0f; // Previous error

    public PIDController(float kp, float ki, float kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public float Control(float setpoint, float actualVelocity)
    {
        // Calculate the error between desired setpoint and actual velocity
        float error = setpoint - actualVelocity;

        // Proportional term
        float proportionalTerm = kp * error;

        // Integral term
        integralError += error * Time.fixedDeltaTime;
        float integralTerm = ki * integralError;

        // Derivative term
        float derivativeTerm = kd * (error - previousError) / Time.fixedDeltaTime;
        previousError = error;

        // Calculate the torque output using the PID controller
        float torque = proportionalTerm + integralTerm + derivativeTerm;

        return torque;
    }

    // This is useful if you want to reset the accumulated error and previous error under certain conditions
    public void Reset()
    {
        integralError = 0f;
        previousError = 0f;
    }
}


