
#include "move.h"

void MoveInstruction::to_char_array(std::vector<char> *output)
{
    output->reserve(output->size() + sizeof(int) + sizeof(speed) + sizeof(target) + sizeof(turn) + sizeof(collision) + sizeof(lift));
    char *p = (char *)(&type);
    for (int i = 0; i < sizeof(int); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&speed);
    for (int i = 0; i < sizeof(speed); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&target);
    for (int i = 0; i < sizeof(target); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&turn);
    for (int i = 0; i < sizeof(turn); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&collision);
    for (int i = 0; i < sizeof(collision); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&lift);
    for (int i = 0; i < sizeof(lift); i++)
    {
        output->push_back(p[i]);
    }
}

void MoveInstruction::from_char_array(char *buf, int size)
{
    if (size >= sizeof(int) + sizeof(speed) + sizeof(target) + sizeof(turn) + sizeof(collision) + sizeof(lift))
    {
        type = *((int *)buf);
        speed = *((struct movement_instruction_speed *)(buf + sizeof(int)));
        target = *((struct movement_instruction_target *)(buf + sizeof(int) + sizeof(speed)));
        turn = *((struct movement_instruction_turn *)(buf + sizeof(int) + sizeof(speed) + sizeof(target)));
        collision = *((struct movement_instruction_collision *)(buf + sizeof(int) + sizeof(speed) + sizeof(target) + sizeof(turn)));
        lift = *((struct movement_instruction_lift *)(buf + sizeof(int) + sizeof(speed) + sizeof(target) + sizeof(turn) + sizeof(collision)));
    }
}
