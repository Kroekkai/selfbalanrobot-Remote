PID Control Implementation: Developed a robust real-time balancing algorithm using Proportional-Integral-Derivative (PID) control to maintain the robot’s equilibrium based on pitch data from an MPU6050 sensor.

Sensor Fusion & Data Filtering: Integrated an MPU6050 (Accelerometer + Gyroscope) via I2C communication, implementing gyro offset calibration for precise tilt angle measurement.

Remote Control Systems: Engineered a dual-control interface featuring IR Remote sensing for manual maneuvering (rotation and speed control) and a Serial command system for real-time PID tuning.

Motor Drive Logic: Programmed PWM (Pulse Width Modulation) signals to control DC motors via an L298N/L293D driver, featuring dead-zone handling and output constraining to ensure hardware longevity.

Serial Communication Protocol: Developed a custom command parser to receive and update control parameters (Kp, Ki, Kd) on-the-fly via Serial Monitor/Python scripts for efficient debugging.
