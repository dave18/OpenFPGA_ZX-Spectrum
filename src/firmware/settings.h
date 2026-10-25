#ifndef SETTINGS_H
#define SETTINGS_H

#define SETTINGS_SIZE 128
#define SETTINGS_PREFS 21
#define SETTINGS_KEYS 10
/* Version 2 stores actual x/y pairs, not experimental v1 display triples. */
#define SETTINGS_VERSION 2
#define SETTINGS_INVALID 0
#define SETTINGS_VALID 1
#define SETTINGS_UNSUPPORTED 2

struct settings_key { unsigned char x,y; };
struct settings {
	unsigned char prefs[SETTINGS_PREFS];
	struct settings_key keys[SETTINGS_KEYS];
};

extern const unsigned char settings_limits[SETTINGS_PREFS];
void settings_encode(unsigned char record[SETTINGS_SIZE],const struct settings *value);
int settings_decode(struct settings *value,const unsigned char *record,unsigned int size);
void settings_key_label(unsigned int x,unsigned int y,unsigned int *label_x,unsigned int *width);

void settings_begin(void);
void settings_default_key(unsigned int id,unsigned int x,unsigned int y);
void settings_restore(void);
void settings_preference_changed(unsigned int menu,unsigned int item);
void settings_key_selected(unsigned int id,unsigned int x,unsigned int y);
void settings_model_selected(unsigned int action);
/* Implemented beside the original matrix routines in main.c. */
void settings_apply_key(unsigned int id,unsigned int x,unsigned int y);

#endif
