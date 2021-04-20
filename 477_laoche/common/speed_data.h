#ifndef AGV_SPEEDD_H
#define AGV_SPEEDD_H
#include <cmath>
#include <vector>

class Speed_data {
public:
    Speed_data() : Speed_data(0, 0.) {};
    Speed_data(double vl, double va)
        : vel_line(vl), vel_ang(va){
        while(vel_ang <- M_PI ) {
            vel_ang += (M_PI * 2);
        }
        while(vel_ang > M_PI ) {
            vel_ang -= (M_PI * 2);
        }
    };

    Speed_data & operator=(const Speed_data &p) {
        vel_line = p.vel_line;
        vel_ang = p.vel_ang;
        return *this;
    }
    double  vel_line;
    double  vel_ang;
    void to_char_array(std::vector<char> *output) {
        output->reserve(output->size() + sizeof(Speed_data));
        char *p = (char*)this;
        for(int i = 0;i < sizeof(Speed_data);i++) {
            output->push_back(p[i]);
        }
    }
    void from_char_array(char *buf, int size) {
        if(size >= sizeof(Speed_data)) {
            Speed_data *p = (Speed_data*)buf;
            vel_line = p->vel_line;
            vel_ang = p->vel_ang;
        }
    }
};
#endif

