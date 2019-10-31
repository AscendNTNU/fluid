#ifndef FLUID_PID_H
#define FLUID_PID_H

namespace fluid {
    
    class PID {

        private: 
            const double kp, ki, kd;

            double integrated_error = 0;
            double previous_error = 0;

        public:

            PID(const double& kp, const double& ki, const double& kd);

            double getActuation(const double& error, const double& delta_time);
    };
}

#endif
