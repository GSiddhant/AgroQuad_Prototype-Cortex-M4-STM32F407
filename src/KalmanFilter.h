
#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

class KalmanFilter
{
public:

    float Qa;
    float Qb;
    float R;
    float angle;
    float bias;
    float rate;
    float P[2][2];

    KalmanFilter()
	{
    	    Qa = 0.001;
    	    Qb = 0.003;
    	    R = 0.03;

    	    angle = 0;
    	    bias = 0;

    	    P[0][0] = 0;
    	    P[0][1] = 0;
    	    P[1][0] = 0;
    	    P[1][1] = 0;
    }

    float Angle(float Angle, float Rate, float dt)
    {
        rate = Rate - bias;
        angle += dt * rate;

        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Qa);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Qb * dt;

        float S = P[0][0] + R;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        float y = Angle - angle;

        angle += K[0] * y;
        bias += K[1] * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }

};




#endif /* KALMANFILTER_H_ */
