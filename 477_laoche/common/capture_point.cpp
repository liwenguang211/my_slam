#include "capture_point.h"

void CapturePoint::from_char_array(char *buf, int size)
{
    if (size < sizeof(int) * 3 + sizeof(double) * 3 + sizeof(extra))
    {
        return;
    }
    buf = buf + sizeof(int);
    index = *((int *)buf);
    int len = *((int *)(buf + sizeof(int)));
    if (name)
    {
        delete name;
    }
    name = new char[len];
    char *p = buf + sizeof(int) * 2;
    for (int i = 0; i < len; i++)
    {
        name[i] = p[i];
    }
    p = p + len;
    double *p2 = (double *)p;
    x = p2[0];
    y = p2[1];
    theta = p2[2];
    struct CapturePointExtraInfo * p3 = (struct CapturePointExtraInfo *)(p + sizeof(double)*3);
    extra = *p3;
    
}

void CapturePoint::to_char_array(std::vector<char> *output)
{
    int len = 0;
    for (len = 0; name[len] != 0; len++)
        ;
    len++;
    int size = len + sizeof(int) * 2 + sizeof(double) * 3 + sizeof(extra);
    output->reserve(output->size() + size);
    char *p = (char *)(&size);
    for (int i = 0; i < sizeof(int); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&index);
    for (int i = 0; i < sizeof(int); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&len);
    for (int i = 0; i < sizeof(int); i++)
    {
        output->push_back(p[i]);
    }
    p = name;
    for (int i = 0; i < len; i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&x);
    for (int i = 0; i < sizeof(double); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&y);
    for (int i = 0; i < sizeof(double); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&theta);
    for (int i = 0; i < sizeof(double); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&extra);
    for (int i = 0; i < sizeof(extra); i++)
    {
        output->push_back(p[i]);
    }
}

void CapturePoint::rename(char *new_name) {
    int len = 0;
    for (len = 0; new_name[len] != 0; len++)
        ;
    len++;
    if (name)
    {
        delete name;
    }
    name = new char[len];
    for (int i = 0; i < len; i++)
    {
        name[i] = new_name[i];
    }
}
