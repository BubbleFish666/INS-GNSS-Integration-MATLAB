%% initialize an IMU model
IMUModel = imuSensor('accel-gyro');
IMUModel.SampleRate = 40;  % Hz
% gyroscope noise
IMUModel.Gyroscope.MeasurementRange = deg2rad(2000);  % rad/s, according to MTi-3
IMUModel.Gyroscope.BiasInstability = 4.848e-5;  % rad/s, according to MTi-3 10 deg/s
IMUModel.Gyroscope.NoiseDensity = deg2rad(0.007);  % (rad/s)/√Hz, according to MTi-3
IMUModel.Gyroscope.RandomWalk = deg2rad(0.007);  % (rad/s)/√Hz, improvised
% accelerometer noise
IMUModel.Accelerometer.MeasurementRange = 16 * 10;  % m/s^2, according to MTi-3
IMUModel.Accelerometer.BiasInstability = 0.03e-3 * 10;  % m/s^2, according to MTi-3
IMUModel.Accelerometer.NoiseDensity = 120e-6 * 10;  % (m/s^2/√Hz), according to MTi-3 120 ug/√Hz
IMUModel.Accelerometer.RandomWalk = 120e-6 * 10;  % (m/s^2/√Hz), improvised
