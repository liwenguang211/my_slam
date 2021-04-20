#ifndef AGV_POSITION_H
#define AGV_POSITION_H
#include <cmath>
#include <vector>

class Position {
public:
    Position() : Position(0, 0., 0., 0.) {};
    Position(long time_us, double x, double y, double theta)
        :timestamp(time_us), x(x), y(y), theta(theta){
        while(theta < -M_PI) {
            theta += (M_PI * 2);
        }
        while(theta > M_PI ) {
            theta -= (M_PI * 2);
        }
    };

    Position & operator=(const Position &p) {
        timestamp = p.timestamp;
        x = p.x;
        y = p.y;
        theta = p.theta;
	delt_t = p.delt_t;
        return *this;
    }

    long    timestamp;
    double  x;
    double  y;
    double  theta;
    long  delt_t;
    double norm() {
        double t = theta;
        if(theta > M_PI) {
            t = M_PI * 2 - theta;
        }
        return x + y + t;
    };

    void to_char_array(std::vector<char> *output) {
        output->reserve(output->size() + sizeof(Position));
        char *p = (char*)this;
        for(int i = 0;i < sizeof(Position);i++) {
            output->push_back(p[i]);
        }
    }
    void from_char_array(char *buf, int size) {
        if(size >= sizeof(Position)) {
            Position *p = (Position*)buf;
            timestamp = p->timestamp;
            x = p->x;
            y = p->y;
            theta = p->theta;
	           delt_t =  p->delt_t ;
        }
    }
};

Position operator-(const Position& a, const Position& b);

Position operator+(const Position& a, const Position& b);

Position operator*(const Position& a, const Position& b);

Position operator/(const Position& p, const Position& a);

bool operator<(const Position& l, const Position& r);
bool operator==(const Position& a, const Position& b);

#endif

