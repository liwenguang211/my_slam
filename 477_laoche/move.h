#ifndef AGV_MOVE_H
#define AGV_MOVE_H

#include <vector>

#define MOVE_INSTRUCTION_TYPE_SPEED 1
#define MOVE_INSTRUCTION_TYPE_TARGET 2
#define MOVE_INSTRUCTION_TYPE_TURN 3
#define MOVE_INSTRUCTION_TYPE_COLLISION 4
#define MOVE_INSTRUCTION_TYPE_LIFT 5
#define MOVE_INSTRUCTION_TYPE_LIFT_HEIGHT 6

struct movement_instruction_target
{
    float start_x;
    float start_y;
    float start_theta;
    float x;
    float y;
    float theta;
	int dir;
};
struct movement_instruction_turn
{
    int dir;
    float angle;
};
struct movement_instruction_speed
{
    int speed;
    int turn_left;
};
struct movement_instruction_collision
{
    int level;
};
struct movement_instruction_lift
{
    int dir;
	int height;
};

class MoveInstruction
{
public:
    int type;
    struct movement_instruction_speed speed;
    struct movement_instruction_target target;
    struct movement_instruction_turn turn;
	struct movement_instruction_collision collision;
    struct movement_instruction_lift lift;

    void to_char_array(std::vector<char> *output);
    void from_char_array(char *buf, int size);
};

#endif
