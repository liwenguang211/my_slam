#ifndef SERILIZABLE_DATA_H
#define SERILIZABLE_DATA_H

#include <vector>

class Serializable {
public:
    virtual void to_char_array(std::vector<char> *output) = 0;
    virtual void from_char_array(char *buf, int size) = 0;
};

#endif


