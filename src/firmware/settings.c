#include "settings.h"

/* Keyboard, joystick, mouse, CPU, GS, mix, PSG/FM, PSG stereo, PSG model,
 * FE, FF, ULA+, snow, timings, memory, MMC mode/version, border, LED, fast
 * tape, tape sound. The original Pocket menu exposes only Auto MMC mode. */
const unsigned char settings_limits[SETTINGS_PREFS] = {
	4,5,3,5,4,4,2,2,2,2,2,2,2,3,5,1,3,2,2,2,2
};

static unsigned int crc32(const unsigned char *record)
{
	unsigned int crc=0xffffffffu;
	for (unsigned int i=0;i<124;i++) {
		crc^=record[i];
		for (unsigned int b=0;b<8;b++)
			crc=(crc>>1) ^ (0xedb88320u & (0u-(crc&1)));
	}
	return ~crc;
}

void settings_encode(unsigned char record[SETTINGS_SIZE],const struct settings *value)
{
	for (unsigned int i=0;i<SETTINGS_SIZE;i++) record[i]=0;
	record[0]='Z'; record[1]='X'; record[2]='C'; record[3]='F';
	record[4]=SETTINGS_VERSION;
	record[6]=SETTINGS_SIZE;
	for (unsigned int i=0;i<SETTINGS_PREFS;i++) record[8+i]=value->prefs[i];
	for (unsigned int i=0;i<SETTINGS_KEYS;i++) {
		record[32+i*2]=value->keys[i].x;
		record[33+i*2]=value->keys[i].y;
	}
	unsigned int crc=crc32(record);
	for (unsigned int i=0;i<4;i++) record[124+i]=(unsigned char)(crc>>(i*8));
}

int settings_decode(struct settings *value,const unsigned char *record,unsigned int size)
{
	if (size<8 || record[0]!='Z' || record[1]!='X' || record[2]!='C' || record[3]!='F')
		return SETTINGS_INVALID;
	/* Never reinterpret or overwrite an experimental/unknown format. */
	if (record[4]!=SETTINGS_VERSION || record[5]) return SETTINGS_UNSUPPORTED;
	if (size!=SETTINGS_SIZE || record[6]!=SETTINGS_SIZE || record[7]) return SETTINGS_INVALID;
	unsigned int crc=0;
	for (unsigned int i=0;i<4;i++) crc|=(unsigned int)record[124+i]<<(i*8);
	if (crc!=crc32(record)) return SETTINGS_INVALID;
	for (unsigned int i=0;i<SETTINGS_PREFS;i++)
		if (record[8+i]>=settings_limits[i]) return SETTINGS_INVALID;
	for (unsigned int i=0;i<SETTINGS_KEYS;i++)
		if (record[32+i*2]>=14 || record[33+i*2]>=5) return SETTINGS_INVALID;
	for (unsigned int i=0;i<SETTINGS_PREFS;i++) value->prefs[i]=record[8+i];
	for (unsigned int i=0;i<SETTINGS_KEYS;i++) {
		value->keys[i].x=record[32+i*2];
		value->keys[i].y=record[33+i*2];
	}
	return SETTINGS_VALID;
}

void settings_key_label(unsigned int x,unsigned int y,unsigned int *label_x,unsigned int *width)
{
	*label_x=x;
	*width=1;
	/* Break/Edit (row 0, columns 0/1) are distinct bindings, not a glyph pair. */
	if ((y==1 && x<4) || (y==2 && x<2) || (y==3 && x<4) ||
		((y==0 || y==2 || y==3) && x>=12)) {
		*label_x=x & ~1u;
		*width=2;
	}
	if (y==4 && x>=5 && x<=9) { *label_x=5; *width=5; }
}

#ifndef SETTINGS_CODEC_ONLY
#include "host.h"
extern uint32_t menuListVals[21];
extern uint32_t menuType[6][15];
static struct settings saved;
static unsigned int writable,ready;
static const unsigned char list_ids[17]={0,1,2,3,8,9,10,11,12,13,14,15,16,17,18,19,20};
static const unsigned char toggle_menu[4]={1,1,5,5};
static const unsigned char toggle_item[4]={1,7,4,5};

#ifdef SETTINGS_TEST
extern unsigned int settings_test_read(unsigned int offset);
extern void settings_test_write(unsigned int offset,unsigned int value);
#define READ_PORT(x) settings_test_read(x)
#define WRITE_PORT(x,v) settings_test_write(x,v)
#else
#define READ_PORT(x) ((x)==128 ? SETTINGS_CONTROL : SETTINGS_WORD(x))
#define WRITE_PORT(x,v) do { if ((x)==128) SETTINGS_CONTROL=(v); else SETTINGS_WORD(x)=(v); } while (0)
#endif

static void publish(void)
{
	unsigned char record[SETTINGS_SIZE];
	if (!ready || !writable) return;
	settings_encode(record,&saved);
	while (READ_PORT(128)&2) {}
	unsigned int token=(READ_PORT(128)&1)^1;
	for (unsigned int i=0;i<SETTINGS_SIZE;i+=4) {
		unsigned int word=0;
		for (unsigned int j=0;j<4;j++) word|=(unsigned int)record[i+j]<<(j*8);
		WRITE_PORT(i,word);
	}
	WRITE_PORT(128,token);
	while (READ_PORT(128)&2) {}
}

void settings_begin(void)
{
	/* The original startup does not initialize .data/BSS on a soft restart.
	 * Initialize ALL private state explicitly; leave its runtime unchanged. */
	ready=0;
	writable=1;
	for (unsigned int i=0;i<SETTINGS_PREFS;i++) saved.prefs[i]=0;
	for (unsigned int i=0;i<SETTINGS_KEYS;i++) { saved.keys[i].x=0; saved.keys[i].y=0; }
}

void settings_default_key(unsigned int id,unsigned int x,unsigned int y)
{
	if (id<SETTINGS_KEYS) { saved.keys[id].x=x; saved.keys[id].y=y; }
}

void settings_restore(void)
{
	unsigned char record[SETTINGS_SIZE];
	struct settings restored;
	for (unsigned int i=0;i<17;i++) saved.prefs[i]=menuListVals[list_ids[i]];
	for (unsigned int i=0;i<4;i++)
		saved.prefs[17+i]=menuType[toggle_menu[i]][toggle_item[i]]>>24;
	for (unsigned int i=0;i<SETTINGS_SIZE;i+=4) {
		unsigned int word=READ_PORT(i);
		for (unsigned int j=0;j<4;j++) record[i+j]=(unsigned char)(word>>(j*8));
	}
	int result=settings_decode(&restored,record,SETTINGS_SIZE);
	writable=(result!=SETTINGS_UNSUPPORTED);
	if (result==SETTINGS_VALID) {
		/* No libc is linked by the original firmware. Volatile reads keep
		 * LLVM's loop distributor from replacing this with memcpy. */
		volatile const struct settings *loaded=&restored;
		for (unsigned int i=0;i<17;i++) {
			saved.prefs[i]=loaded->prefs[i];
			menuListVals[list_ids[i]]=saved.prefs[i];
		}
		for (unsigned int i=0;i<4;i++) {
			unsigned int m=toggle_menu[i],item=toggle_item[i];
			saved.prefs[17+i]=loaded->prefs[17+i];
			menuType[m][item]=(menuType[m][item]&0xffffff) |
				((uint32_t)saved.prefs[17+i]<<24);
		}
		for (unsigned int i=0;i<SETTINGS_KEYS;i++) {
			settings_default_key(i,loaded->keys[i].x,loaded->keys[i].y);
			settings_apply_key(i,saved.keys[i].x,saved.keys[i].y);
		}
	}
	ready=1;
	publish();
}

void settings_preference_changed(unsigned int menu,unsigned int item)
{
	if (menu>=6 || item>=15) return;
	unsigned int type=(menuType[menu][item]>>8)&0xff;
	if (type==2) {
		unsigned int list=menuType[menu][item]>>24;
		for (unsigned int i=0;i<17;i++)
			if (list==list_ids[i] && menuListVals[list]<settings_limits[i])
				saved.prefs[i]=menuListVals[list];
	} else if (type==3 || type==4) {
		for (unsigned int i=0;i<4;i++)
			if (menu==toggle_menu[i] && item==toggle_item[i])
				saved.prefs[17+i]=(menuType[menu][item]>>24)&1;
	} else return;
	publish();
}

void settings_key_selected(unsigned int id,unsigned int x,unsigned int y)
{
	if (id>=SETTINGS_KEYS || x>=14 || y>=5) return;
	settings_default_key(id,x,y);
	publish();
}

void settings_model_selected(unsigned int action)
{
	/* Same actions as the original Fn-key dispatch; do not change dispatch,
	 * or turn snapshot-driven runtime overrides into preferred startup modes. */
	switch (action) {
		case 0x39: saved.prefs[13]=0; saved.prefs[14]=3; break;
		case 0x3a: saved.prefs[13]=1; saved.prefs[14]=0; break;
		case 0x3b: saved.prefs[13]=1; saved.prefs[14]=4; break;
		case 0x3c: saved.prefs[13]=2; saved.prefs[14]=3; break;
		case 0x3d: saved.prefs[13]=2; saved.prefs[14]=0; break;
		case 0x3e: saved.prefs[13]=2; saved.prefs[14]=1; break;
		default: return;
	}
	publish();
}
#endif
