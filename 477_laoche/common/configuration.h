#ifndef AGV_CONFIGURATION_H
#define AGV_CONFIGURATION_H

#include <string>
#include <map>

#define GET_CONFIG_INT(name, t, i) int name = get_configs()->get_int(t, i, nullptr);
#define GET_CONFIG_BOOL(name, t, i) bool name = get_configs()->get_bool(t, i, nullptr);
#define GET_CONFIG_LONG(name, t, i) long name = get_configs()->get_long(t, i, nullptr);
#define GET_CONFIG_FLOAT(name, t, i) float name = get_configs()->get_float(t, i, nullptr);
#define GET_CONFIG_DOUBLE(name, t, i) double name = get_configs()->get_float(t, i, nullptr);


typedef enum {t_char = 0, t_bool, t_int, t_long, t_float, t_str, t_err} ConfigItemType;

class ConfigItem {
public:
    ConfigItemType type;
    std::string value;
};

class Configuration {
public:
    Configuration(char * file_path);
    ~Configuration() {};

    bool get_bool(char * table, char *item, int *err);
    int get_int(char * table, char *item, int *err);
    long get_long(char * table, char *item, int *err);
    float get_float(char * table, char *item, int *err);
    void get_string(char * table, char *item, char *buf, int *err);

    void print();

private:
    std::map<std::string, std::map<std::string, ConfigItem>> dictionary;

    ConfigItem get(char * table, char *item);
    std::string get_string(char * table, char *item, int *err);
};

void load_config(char * file_path);

Configuration * get_configs();

#endif
