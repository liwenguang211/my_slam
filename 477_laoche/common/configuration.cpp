#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include "configuration.h"

void to_low_case(char * str) {
    int pos = 0;
    while(str[pos] != 0) {
        if(str[pos] >= 'A' && str[pos] <= 'Z') {
            str[pos] += ('a' - 'A');
        }
        pos ++;
    }
}

bool str_equel(const char *str1, const char *str2) {
    int pos = 0;
    while(1) {
        if(str1[pos] == 0 || str2[pos] == 0) {
            if(str1[pos] == 0 && str2[pos] == 0) {
                return true;
            }
            else {
                return false;
            }
        }
        if(str1[pos] != str2[pos]) {
            return false;
        }
        pos++;
    }
}

void split_str(const char *str, std::vector<std::string> *words) {
    int pos = 0;
    std::string tmp;
    while(1) {
        char c = str[pos++];
        if(c == 0 || c == ' ' || c == '\n' || c == '\t' || c == ':') {
            if(tmp.size() > 0) {
                words->push_back(tmp);
                tmp.clear();
            }
        }
        else if(c == '"') {
            while((c = str[pos++]) != '"') {
                tmp.push_back(c);
            }
        }
        else {
            tmp.push_back(c);
        }
        if(c == 0) {
            if(tmp.size() > 0) {
                words->push_back(tmp);
                tmp.clear();
            }
            return;
        }
        if(c == ':') {
            words->push_back(std::string(":"));
        }
    }
}

char *type_str[] = {
    "char",
    "bool",
    "int",
    "long",
    "float",
    "string"
};

ConfigItemType get_type(const char *str) {
    if(str_equel(str, type_str[0])) {
        return t_char;
    }
    if(str_equel(str, type_str[1])) {
        return t_bool;
    }
    if(str_equel(str, type_str[2])) {
        return t_int;
    }
    if(str_equel(str, type_str[3])) {
        return t_long;
    }
    if(str_equel(str, type_str[4])) {
        return t_float;
    }
    if(str_equel(str, type_str[5])) {
        return t_str;
    }
    return t_err;
}

Configuration::Configuration(char * file_path) {
    FILE *fp = fopen(file_path, "r");
    char buf[512];
    printf("open configuration file : %s\n", file_path);

    std::string current_table("");
    std::map<std::string, ConfigItem> current_map;

    std::vector<std::string> words;
    while(1) {
        if(fgets(buf, 512, (FILE*)fp) == nullptr) {
            if(current_map.size() > 0) {
                dictionary.insert(std::pair<std::string, std::map<std::string, ConfigItem>>(current_table, current_map));
                // printf("Insert %s: \n", current_table.c_str());
                for(auto & e2 : current_map) {
                    // printf("\t%s %d %s \n", e2.first.c_str(), e2.second.type, e2.second.value.c_str());
                }
            }
            break;
        };
        to_low_case(buf);
        words.clear();
        split_str(buf, &words);
        if(words.size() < 2 || words[0][0] == '#') {
            continue;
        }
        if(words[1][0] == ':') {
            // printf("Get table name %s\n", words[0].c_str());
            if(current_map.size() > 0) {
                dictionary.insert(std::pair<std::string, std::map<std::string, ConfigItem>>(current_table, current_map));
                // printf("Insert %s: \n", current_table.c_str());
                for(auto & e2 : current_map) {
                    // printf("\t%s %d %s \n", e2.first.c_str(), e2.second.type, e2.second.value.c_str());
                }
            }
            current_map.clear();
            current_table = words[0];
            continue;
        }
        if(words.size() >= 3) {
            ConfigItemType t = get_type(words[1].c_str());
            // printf("Get item %s %s %s\n", words[0].c_str(), words[1].c_str(), words[2].c_str());
            if(t == t_err) {
                // printf("load config error : \n%s : %s %s %s\n", current_table.c_str(), words[0].c_str(), words[1].c_str(), words[2].c_str());
                exit(1);
            }
            ConfigItem item;
            item.type = t;
            item.value = words[2];
            current_map.insert(std::pair<std::string, ConfigItem>(words[0], item));
            continue;
        }
    }

    fclose(fp);
}

void Configuration::print() {
    for(auto & e1 : dictionary) {
        printf("%s :\n", e1.first.c_str());
        for(auto & e2 : e1.second) {
            printf("\t%s %d %s \n", e2.first.c_str(), e2.second.type, e2.second.value.c_str());
        }
    }
}

ConfigItem Configuration::get(char * table, char *item) {
    to_low_case(table);
    to_low_case(item);
    std::string t(table), i(item);
    auto search1 = dictionary.find(t);
    if(search1 != dictionary.end()) {
        auto search2 = search1->second.find(i);
        if(search2 != search1->second.end()) {
            return search2->second;
        }
    }
    ConfigItem ret;
    ret.type = t_err;
    printf("Configuration : Missing item %s/%s\n", table, item);
    exit(1);
    return ret;
}

bool Configuration::get_bool(char * table, char *item, int *err) {
    ConfigItem i = get(table, item);
    if(i.type != t_bool) {
        if(err) {
            *err = 1;
        }
        else {
            printf("Configuration : Expect format (bool): %s/%s\n", table, item);
            exit(1);
        }
        return false;
    }
    if(err) {
        *err = 0;
    }
    if(atoi(i.value.c_str()) != 0) {
        return true;
    }
    return false;
}
int Configuration::get_int(char * table, char *item, int *err) {
    ConfigItem i = get(table, item);
    if(i.type != t_int) {
        if(err) {
            *err = 1;
        }
        else {
            printf("Configuration : Expect format (int): %s/%s\n", table, item);
            exit(1);
        }
        return 0;
    }
    if(err) {
        *err = 0;
    }
    return atoi(i.value.c_str());
}
long Configuration::get_long(char * table, char *item, int *err) {
    ConfigItem i = get(table, item);
    if(i.type != t_long) {
        if(err) {
            *err = 1;
        }
        else {
            printf("Configuration : Expect format (long): %s/%s\n", table, item);
            exit(1);
        }
        return 0;
    }
    if(err) {
        *err = 0;
    }
    return atol(i.value.c_str());
}
float Configuration::get_float(char * table, char *item, int *err) {
    ConfigItem i = get(table, item);
    if(i.type != t_float) {
        if(err) {
            *err = 1;
        }
        else {
            printf("Configuration : Expect format (float): %s/%s\n", table, item);
            exit(1);
        }
        return 0;
    }
    if(err) {
        *err = 0;
    }
    return (float)(atof(i.value.c_str()));
}
std::string Configuration::get_string(char * table, char *item, int *err){
    ConfigItem i = get(table, item);
    if(i.type != t_str) {
        if(err) {
            *err = 1;
        }
        else {
            printf("Configuration : Expect format (string): %s/%s\n", table, item);
            exit(1);
        }
        return 0;
    }
    if(err) {
        *err = 0;
    }
    return i.value;
}
void Configuration::get_string(char * table, char *item, char *buf, int *err) {
    std::string s = get_string(table, item, err);
    strcpy(buf, s.c_str());
    int pos = 0;
    while(buf[pos] != 0) {
        if(buf[pos] < 32) {
            buf[pos] = 0;
            break;
        }
        pos++;
    }
}

Configuration * p_config = nullptr;

void load_config(char * file_path) {
    p_config = new Configuration(file_path);
    // p_config->print();
}


Configuration * get_configs() {
    return p_config;
}

// int main() {
//     Configuration config("config.txt");
//     config.print();
//     printf("item 1_1 : %d\n", config.get_int("test1", "item1_1", nullptr));
//     printf("item 1_2 : %f\n", config.get_float("test1", "item1_2", nullptr));
//     printf("item 1_3 : %d\n", config.get_bool("test1", "item1_3", nullptr));
//     printf("item 2_1 : %ld\n", config.get_long("test2", "item2_1", nullptr));
// }
