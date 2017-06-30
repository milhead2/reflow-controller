#ifndef __DISPLAY_H_INCLUDED__
#define __DISPLAY_H_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_CLEAR (0x01)

void display_init(void);
void display_character(char data_value);
void display_command(char cmd_value);

void display_home(void);
void display_clear(void);
void display_set_cursor(int line, int offset);
void display_clear_line(int line);
void display_string(const char * s);


#ifdef __cplusplus
}
#endif


#endif //  __DISPLAY_H_INCLUDED__
