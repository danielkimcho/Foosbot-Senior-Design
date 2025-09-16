#ifndef __COMMANDS_H__
#define __COMMANDS_H__

struct commands_t {
    const char *cmd;
    void      (*fn)(int argc, char *argv[]);
};

void command_shell(void);
void mount();
int read_first_line();
int write_score_to_file(int score);

#endif /* __COMMANDS_H_ */
