#include <limits.h>
#include "host.h"
#include "main.h"
#include "charfontb.h"

int osd_width;
int osd_char_width;
int osd_height;
int osd_char_height;
int numitems=0;
int cursorpos=0;
int oldpos;
int currmenu=0;
int debug_on;
int key_auto;
int auto_kptr;
int auto_delay;
int cap_shift;
int sym_shift;

uint32_t status_bits[64];
uint32_t status0;
uint32_t status1;

char filename[256];
char ext[4];

int ioctl_index;

char make_upper(char c) {
	if ((c>=0x61) && (c<=0x7a)) c-=0x20;
	return c;
}

void * memset (void *dest, int val, unsigned int len)
{
  register unsigned char *ptr = (unsigned char*)dest;
  while (len-- > 0)
    *ptr++ = val;
  return dest;
}

int memcmp (const void *str1, const void *str2, unsigned int count)
{
  register const unsigned char *s1 = (const unsigned char*)str1;
  register const unsigned char *s2 = (const unsigned char*)str2;

  while (count-- > 0)
    {
      if (*s1++ != *s2++)
	  return s1[-1] < s2[-1] ? -1 : 1;
    }
  return 0;
}

static uint32_t strlen(const char *src)
{
	int i=0;
	while (src[i])
	{
		i=i+1;
	}
	return i;
}


char * strcpyr(char *s1, const char *s2)
{
    char *s = s1;    /* s1 assign address to s */
    while ((*s++ = *s2++) != 0)   /* s++ only the address of s plus one */
    ;
    return (s1);
}

int strcmp(const char *s1, const char *s2)
{
	while (*s1 == *s2++)
		if (*s1++ == '\0')
			return (0);
	return (*(const unsigned char *)s1 - *(const unsigned char *)(s2 - 1));
}

void strrev(char *str)
{
	int i;
	int j;
	unsigned char a;
	unsigned len = strlen((const char *)str);
	for (i = 0, j = len - 1; i < j; i++, j--)
	{
		a = str[i];
		str[i] = str[j];
		str[j] = a;
	}
}

#define INT_DIGITS 19*4		// enough for 64 bit integer 

char itoabuf[INT_DIGITS + 2];


void itoa(int si)       //currently hacked just for hex values
{
  // Room for INT_DIGITS digits, - and '\0' 
  unsigned int i=(unsigned int)si;
  int p=0;    
  if (i==0)	{  //deal with 0 specifically
	itoabuf[0]='0';
	itoabuf[1]=0;
	return;
  }
  //if (i > 0) {
    do {
      if ((i & 0xf) < 0xa) itoabuf[p] = '0' + (i & 0xf); else itoabuf[p] = 'A' + ((i & 0xf)-10);
	  p++;
      i >>= 4;
    } while (i != 0);
	itoabuf[p] = 0;
	strrev(itoabuf);
  //  return;
  
  /*else {			// i < 0 
	i=-i;
    do {
      if ((i & 0xf) < 0xa) itoabuf[p] = '0' + (i & 0xf); else itoabuf[p] = 'A' + ((i & 0xf)-10);
	  p++;
      i >>= 4;
    } while (i != 0);
    itoabuf[p] = '-';
	p++;
	itoabuf[p] = 0;
	strrev(itoabuf);
  }*/
  return;
}


void writeChar(unsigned char c,int address,int inv)
{
	int i;
	unsigned char mask;
	mask=inv?0xff:0x00;
	for (i=0;i<8;i++)	{
		OSD_RW_8(address)=mask ^ charfont[c] [i];
		address+=osd_char_width;
	}
}

void writeString(const char * s,int x, int y)
{	
	if ((x<0) || (y<0)) return;
	x=x >> 3;		//x position must be byte aligned	
	int l=strlen(s);
	if (l<1) return;
	int address;
	int i;
	for (i=0;i<l;i++) {
		address=y*osd_char_width+x+i;
		writeChar(s[i],address,0);
	}
}

void writeHex(int v,int x, int y)
{	
	itoa(v);
	writeString(itoabuf,x,y);
}


uint32_t	readJoypad()
{
	return IO_RW(IO_JOYPAD);
}

uint32_t	readIOCTLState()
{
	return IO_RW(IO_IOCTL_STATE);
}

uint32_t	readIOCTLAddr()
{
	return IO_RW(IO_IOCTL_ADDR);
}

uint32_t	readIOAckflag()
{
	return IO_RW(IO_ACK_FLAG);
}

uint32_t readIOTAPData()
{
	return IO_RW(IO_TAP_DATA);
}

uint32_t readIOTAPAddr()
{
	return IO_RW(IO_TAP_ADDR);
}


void setOSDSize(int w,int h)	
{
	osd_width=w;
	osd_char_width=w>>3;
	osd_height=h;
	osd_char_height=h>>3;
}

int coord2address(int x,int y)
{
	return y*osd_char_width+(x>>3);
}


void drawBorder()
{
	//top left	
	writeChar(134,0,0);
	//top right	
	writeChar(136,osd_char_width-1,0);
	//bottom left	
	writeChar(139,(osd_height-8)*osd_char_width,0);
	//bottom right	
	writeChar(138,(osd_height-8)*osd_char_width+osd_char_width-1,0);
	//top & bottom
	int i;
	for (i=1;i<osd_char_width-1;i++) {		
		
		writeChar(135,i,0);		
		writeChar(135,(osd_height-8)*osd_char_width+i,0);
	}	
	//left & right	
	for (i=1;i<osd_char_height-1;i++) {		
		writeChar(137,i*8*osd_char_width,0);		
		writeChar(137,i*8*osd_char_width+osd_char_width-1,0);		
	}
}

void clearOSD(int foreground)
{
	int y;
	int x;
	for (y=0;y<osd_height;y++)
		for (x=0;x<osd_char_width;x++)
			OSD_RW_8(y*osd_char_width+x)=foreground?0xff:0x00;
}



//************* MENU CODE ********************
char menuItems [MAXMENU] [MAXMENUITEM+1] [MAXMENUITEMLEN+1];
char menuLists [MAXLISTIDS] [MAXLISTITEMS] [MAXLISTLEN+1];
uint32_t menuListVals [MAXLISTIDS];
uint32_t menuType[MAXMENU] [MAXMENUITEM+1];
/*
bits 7-0
id

bits 15-8
0 = HEADER
1 = Sub Menu
2 = List 
3 = Yes/No
4 = On/Off
5 = Special (Map joystick)
9 = Reset

bits 23 - 16
num items in list

bits 31 - 24
list id or menu id



*/



void updateCursor()
{
	writeChar(32,coord2address(8,oldpos*8+16),0);
	writeChar(RIGHT_ARROW,coord2address(8,cursorpos*8+16),0);
	oldpos=cursorpos;
}



void writeTitle(const char *s)
{
	
	//Separation line
	//rotateChar(&charfont[129] [0]);	
	int i;
	for (i=1;i<osd_char_width-1;i++) {		
		
		//writeRotChar(16*osd_char_width+i);
		writeChar(129,16*osd_char_width+i,0);
	}	
	writeString(s,8,8);
}

int menu_on;
int writeMenu()
{
	writeTitle(&menuItems [currmenu] [0] [0]);	
	int y=24;	//starting row
	int i=0;
	int l;
	int v;
	while ((strcmp(&menuItems [currmenu] [i+1] [0],"END") != 0) && (i<MAXMENUITEM+1))
	{
		writeString(&menuItems [currmenu] [i+1] [0],16,y);
		if (((menuType[currmenu] [i+1] >> 8) & 0xff) == 2) { //list			
			v=(menuType[currmenu] [i+1] >> 24) & 0xff; //get list id
			l=strlen(&menuLists [v] [menuListVals[v]] [0]); //get length of text
			writeString(&menuLists [v] [menuListVals[v]] [0],osd_width-(l*8)-8,y);
		}
		if (((menuType[currmenu] [i+1] >> 8) & 0xff) == 3) { //yes/no
			v=(menuType[currmenu] [i+1] >> 24) & 0xff; //get state
			if (v) writeString("Yes",osd_width-(3*8)-8,y); else writeString(" No",osd_width-(3*8)-8,y);
		}
		if (((menuType[currmenu] [i+1] >> 8) & 0xff) == 4) { //on/off		
			v=(menuType[currmenu] [i+1] >> 24) & 0xff; //get state
			if (v) writeString("Off",osd_width-(3*8)-8,y); else writeString(" On",osd_width-(3*8)-8,y);
		}
		i++;
		y+=8;
	}	
	return i;
}


void encode_status() {
	
/* status bits (as per MiSTer)
[1]		Tape Sound,On,Off
[3:2]	Stereo Mix,none,25%,50%,100%
[5:4]	Aspect Ratio,Original,Full Screen,[ARC1],[ARC2];",
[6]		Fast Tape Load,On,Off
[7]		Port #FE,Issue 2,Issue 3
[9:8]	Video Timings,ULA-48,ULA-128,Pentagon
[12:10]	Memory,Spectrum 128K/+2,Pentagon 1024K,Profi 1024K,Spectrum 48K,Spectrum +2A/+3
[13]	Port #FF,Timex,SAA1099
[14]	ULA+,Enabled,Disabled
[16:15]	Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%
[19:17]	Joystick,Kempston,Sinclair I,Sinclair II,Sinclair I+II,Cursor
[21:20]	General Sound,512KB,1MB,2MB,Disabled
[24:22]	CPU Speed,Original,7MHz,14MHz,28MHz,56MHz
[25]	Snow Bug,Disabled,Enabled
[27:26]	Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer
[28]	Vertical Crop,No,Yes
[29:28]	Vertical Crop,No,270,216
[31:30]	MMC Version,DivMMC+ESXDOS,DivMMC,ZXMMC
[33:32]	MMC Mode,Auto(VHD),SD Card 14MHz,SD Card 28MHz
[35:34]	Mouse,Disabled,Kempston L/R,Kempston R/L
[37:36]	Keyboard,Normal,Ghosting,Recreated ZX,Recr+Ghosting
[38]	Narrow Border,No,Yes
[39]	PSG/FM,Enabled,Disabled
[40]	PSG Stereo,ABC,ACB
[41]	PSG Model,YM2149,AY8910

*/
	status_bits[0]=0;		//assume no reset
	status_bits[1]=(menuType[5] [5]>>24) & 0x01;	//tape sound
	status_bits[2]=menuListVals[0x09] & 0x01;		//stereo mix 2-3
	status_bits[3]=(menuListVals[0x09] >>1) & 0x01;
	/*status_bits[4]=menuListVals[0x04] & 0x01;		//aspect ratio 4-5
	status_bits[5]=(menuListVals[0x04] >>1) & 0x01;*/
	status_bits[4]=(menuType[1] [1]>>24) & 0x01;	//Border on/off
	status_bits[6]=(menuType[5] [4]>>24) & 0x01;	//fast tape
	status_bits[7]=menuListVals[0x0d] & 0x01;		//Port #FE 7
	status_bits[8]=menuListVals[0x11] & 0x01;		//video timing 8-9
	status_bits[9]=(menuListVals[0x11] >>1) & 0x01;
	status_bits[10]=menuListVals[0x12] & 0x01;		//Memory 10-12
	status_bits[11]=(menuListVals[0x12] >>1) & 0x01;
	status_bits[12]=(menuListVals[0x12] >>2) & 0x01;
	status_bits[13]=menuListVals[0x0e] & 0x01;		//Port #FF 13
	status_bits[14]=menuListVals[0x0f] & 0x01;		//ULA+ 14
	/*status_bits[15]=menuListVals[0x05] & 0x01;		//Scandoubler 15-16
	status_bits[16]=(menuListVals[0x05] >>1) & 0x01;*/
	status_bits[17]=menuListVals[0x01] & 0x01;		//joystick 17-19
	status_bits[18]=(menuListVals[0x01] >>1) & 0x01;
	status_bits[19]=(menuListVals[0x01] >>2) & 0x01;
	status_bits[20]=menuListVals[0x08] & 0x01;		//General Sound 20-21
	status_bits[21]=(menuListVals[0x08] >>1) & 0x01;
	status_bits[22]=menuListVals[0x03] & 0x01;		//cpu speed 22-24
	status_bits[23]=(menuListVals[0x03] >>1) & 0x01;
	status_bits[24]=(menuListVals[0x03] >>2) & 0x01;
	status_bits[25]=menuListVals[0x10] & 0x01;		//snow bug 25
	/*status_bits[26]=menuListVals[0x07] & 0x01;		//scale 26-27
	status_bits[27]=(menuListVals[0x07] >>1) & 0x01;
	status_bits[28]=menuListVals[0x06] & 0x01;		//vertical crop 28-29
	status_bits[29]=(menuListVals[0x06] >>1) & 0x01;*/
	status_bits[30]=menuListVals[0x14] & 0x01;		//mmc version 30-31
	status_bits[31]=(menuListVals[0x14] >>1) & 0x01;
	status_bits[32]=menuListVals[0x13] & 0x01;		//mmc mode 32-33
	status_bits[33]=(menuListVals[0x13] >>1) & 0x01;
	status_bits[34]=menuListVals[0x02] & 0x01;		//mouse 34-35
	status_bits[35]=(menuListVals[0x02] >>1) & 0x01;
	status_bits[36]=menuListVals[0x00] & 0x01;		//keyboard 36-37
	status_bits[37]=(menuListVals[0x00] >>1) & 0x01;
	//status_bits[38]=(menuType[1] [3]>>24) & 0x01;	//narrow border 38
	status_bits[39]=menuListVals[0x0a] & 0x01;		//PSG/FM 39
	status_bits[40]=menuListVals[0x0b] & 0x01;		//PSG Stereo 40
	status_bits[41]=menuListVals[0x0c] & 0x01;		//PSG Model 41
	status_bits[42]=(menuType[1] [7]>>24) & 0x01;	//Disk LED
	
	uint32_t i;
	
	for (i=43;i<64;i++) { //zero unused bits
		status_bits[i]=0;
	}
	
	status0=0;
	status1=0;
	for (i=0;i<32;i++) {		//shift bits into position
		status0 |= (status_bits[i] << i);
		status1 |= (status_bits[i+32] << i);
	}
	

}

uint32_t updateMenuType(uint32_t old, uint8_t value) {
	old &= 0x00ffffff;		//clear old type byte
	old |= (value <<24);	//or in new value
	return old;
}

void decode_status() {

	uint32_t i;
		

	for (i=0;i<32;i++) {		//shift bits into position		
		status_bits[i]=(status0 >> i) & 1;
		status_bits[i+32]=(status1 >> i) & 1;
	}	
	
	menuType[5] [5]=updateMenuType(menuType[5] [5],status_bits[1]);	//tape sound
	menuListVals[0x09]=status_bits[2] | (status_bits[3] <<1);		//stereo mix 2-3
	menuType[1] [1]=updateMenuType(menuType[1] [1],status_bits[4]);	//border on/off 4-5
	menuType[5] [4]=updateMenuType(menuType[5] [4],status_bits[6]);	//fast tape
	menuListVals[0x0d]=status_bits[7]; 								//Port #FE 7
	menuListVals[0x11]=status_bits[8] | (status_bits[9] <<1);		//video timing 8-9	
	menuListVals[0x12]=status_bits[10] | (status_bits[11] <<1) | (status_bits[12] <<2);				//Memory 10-12	
	menuListVals[0x0e]=status_bits[13]; 							//Port #FF 13
	menuListVals[0x0f]=status_bits[14]; 							//ULA+ 14
	//menuListVals[0x05]=status_bits[15] | (status_bits[16] <<1);		//Scandoubler 15-16	
	menuListVals[0x01]=status_bits[17] | (status_bits[18] <<1) | (status_bits[19] <<2);		//joystick 17-19
	menuListVals[0x08]=status_bits[20] | (status_bits[21] <<1);		//General Sound 20-21	
	menuListVals[0x03]=status_bits[22] | (status_bits[23] <<1) | (status_bits[24] <<2);		//cpu speed 22-24	
	menuListVals[0x10]=status_bits[25]; 							//snow bug 25
	//menuListVals[0x07]=status_bits[26] | (status_bits[27] <<1);		//scale 26-27	
	//menuListVals[0x06]=status_bits[28] | (status_bits[29] <<1);		//vertical crop 28-29	
	menuListVals[0x14]=status_bits[30] | (status_bits[31] <<1);		//mmc version 30-31	
	menuListVals[0x13]=status_bits[32] | (status_bits[33] <<1);		//mmc mode 32-33	
	menuListVals[0x02]=status_bits[34] | (status_bits[35] <<1);		//mouse 34-35	
	menuListVals[0x00]=status_bits[36] | (status_bits[37] <<1);		//keyboard 36-37	
	//menuType[1] [3]=updateMenuType(menuType[1] [3],status_bits[38]);	//narrow border 38
	menuListVals[0x0a]=status_bits[39]; 							//PSG/FM 39
	menuListVals[0x0b]=status_bits[40]; 							//PSG Stereo 40
	menuListVals[0x0c]=status_bits[41]; 							//PSG Model 41
	menuType[1] [7]=updateMenuType(menuType[1] [7],status_bits[42]);	//Disk LED on/off
	

	if (menu_on) writeMenu();

}






/*
bits 7-0
id

bits 15-8
0 = HEADER
1 = Sub Menu
2 = List 
3 = Yes/No
4 = On/Off
5 = Special (Map joystick)
6 = Reset F Keys
7 = Model F Keys
8 = Tape Optiona
9 = Reset

bits 23 - 16
num items in list

bits 31 - 24
list id or menu id



*/
void initMenus() {
	strcpyr(&menuItems[0] [0] [0],"MAIN MENU");
	strcpyr(&menuItems[0] [1] [0],"Audio & Video");
	strcpyr(&menuItems[0] [2] [0],"Hardware");
	strcpyr(&menuItems[0] [3] [0],"Keyboard:");
	strcpyr(&menuItems[0] [4] [0],"Joystick:");
	strcpyr(&menuItems[0] [5] [0],"Map Keyboard Joystick");
	strcpyr(&menuItems[0] [6] [0],"Mouse:");
	strcpyr(&menuItems[0] [7] [0],"Tape Options");
	//strcpyr(&menuItems[0] [8] [0],"Tape Sound:");
	strcpyr(&menuItems[0] [8] [0],"CPU Speed:");
	strcpyr(&menuItems[0] [9] [0],"Reset & Apply");
	strcpyr(&menuItems[0] [10] [0],"Other Reset Options");
	strcpyr(&menuItems[0] [11] [0],"Quick Model Select");
	strcpyr(&menuItems[0] [12] [0],"END");
	
	menuType[0] [0] = 0x00000001;
	menuType[0] [1] = 0x01000102;
	menuType[0] [2] = 0x02000103;
	menuType[0] [3] = 0x00040204;
	strcpyr(&menuLists[0x00] [0] [0],"    Normal");
	strcpyr(&menuLists[0x00] [1] [0],"  Ghosting");
	strcpyr(&menuLists[0x00] [2] [0]," Recreated");
	strcpyr(&menuLists[0x00] [3] [0],"Recr+Ghost");
	menuListVals[0x00]=0;
	
	menuType[0] [4] = 0x01050205;
	strcpyr(&menuLists[0x01] [0] [0],"     Kempston");
	strcpyr(&menuLists[0x01] [1] [0],"   Sinclair I");
	strcpyr(&menuLists[0x01] [2] [0],"  Sinclair II");
	strcpyr(&menuLists[0x01] [3] [0],"       Cursor");
	strcpyr(&menuLists[0x01] [4] [0],"   Key Mapped");
	menuListVals[0x01]=0;
	
	menuType[0] [5] = 0x00000506;		
	
	menuType[0] [6] = 0x02030207;
	strcpyr(&menuLists[0x02] [0] [0],"    Disabled");
	strcpyr(&menuLists[0x02] [1] [0],"Kempston L/R");
	strcpyr(&menuLists[0x02] [2] [0],"Kempston R/L");
	menuListVals[2]=0;
		
	menuType[0] [7] = 0x05000140;	
	
	menuType[0] [8] = 0x0305020a;
	strcpyr(&menuLists[0x03] [0] [0],"Original");
	strcpyr(&menuLists[0x03] [1] [0],"    7MHz");
	strcpyr(&menuLists[0x03] [2] [0],"   14Mhz");
	strcpyr(&menuLists[0x03] [3] [0],"   28Mhz");
	strcpyr(&menuLists[0x03] [4] [0],"   56Mhz");	
	menuListVals[3]=0;
	
	menuType[0] [9] = 0x0000090b;
	
	menuType[0] [10] = 0x03000102;
	menuType[0] [11] = 0x04000103;
	
	//strcpyr(&menuItems[0] [12] [0],"END");
	
	
	strcpyr(&menuItems[1] [0] [0],"AUDIO & VIDEO");
	strcpyr(&menuItems[1] [1] [0],"Border:");
	//strcpyr(&menuItems[1] [2] [0],"Scandoubler Fx:");
	//strcpyr(&menuItems[1] [3] [0],"Narrow Border:");
	//strcpyr(&menuItems[1] [4] [0],"Vertical Crop:");
	//strcpyr(&menuItems[1] [5] [0],"Scale:");
	strcpyr(&menuItems[1] [2] [0],"General Sound:");
	strcpyr(&menuItems[1] [3] [0],"Stereo Mix:");
	strcpyr(&menuItems[1] [4] [0],"PSG/FM:");
	strcpyr(&menuItems[1] [5] [0],"PSG Stereo:");
	strcpyr(&menuItems[1] [6] [0],"PSG Mode:");
	strcpyr(&menuItems[1] [7] [0],"Disk LED:");
	strcpyr(&menuItems[1] [8] [0],"END");
	
	menuType[1] [0] = 0x00000011;
	menuType[1] [1] = 0x00000412;
	/*strcpyr(&menuLists[0x04] [0] [0],"   Original");
	strcpyr(&menuLists[0x04] [1] [0],"Full Screen");
	menuListVals[0x04]=0;*/
	
	/*menuType[1] [2] = 0x05040213;
	strcpyr(&menuLists[0x05] [0] [0],"   None");
	strcpyr(&menuLists[0x05] [1] [0],"   HQ2x");
	strcpyr(&menuLists[0x05] [2] [0],"CRT 25%");
	strcpyr(&menuLists[0x05] [3] [0],"CRT 50%");
	menuListVals[0x05]=0;
	
	menuType[1] [3] = 0x00000314;
	//menuType[1] [4] = 0x00000315
	
	menuType[1] [4] = 0x06030215;
	strcpyr(&menuLists[0x06] [0] [0]," No");
	strcpyr(&menuLists[0x06] [1] [0],"270");
	strcpyr(&menuLists[0x06] [2] [0],"216");
	menuListVals[0x06]=0;
	
	menuType[1] [5] = 0x07040216;
	strcpyr(&menuLists[0x07] [0] [0],"             Normal");
	strcpyr(&menuLists[0x07] [1] [0],"          V-Integer");
	strcpyr(&menuLists[0x07] [2] [0],"Narrower HV-Integer");
	strcpyr(&menuLists[0x07] [3] [0],"   Wider HV-Integer");
	menuListVals[0x07]=0;*/
	
	menuType[1] [2] = 0x08040217;
	strcpyr(&menuLists[0x08] [0] [0],"   512KB");
	strcpyr(&menuLists[0x08] [1] [0],"     1MB");
	strcpyr(&menuLists[0x08] [2] [0],"     2MB");
	strcpyr(&menuLists[0x08] [3] [0],"Disabled");
	menuListVals[0x08]=0;
	
	menuType[1] [3] = 0x09040218;
	strcpyr(&menuLists[0x09] [0] [0],"None");
	strcpyr(&menuLists[0x09] [1] [0]," 25%");
	strcpyr(&menuLists[0x09] [2] [0]," 50%");
	strcpyr(&menuLists[0x09] [3] [0],"100%");
	menuListVals[0x09]=0;
	
	menuType[1] [4] = 0x0a020219;
	strcpyr(&menuLists[0x0a] [0] [0]," Enabled");
	strcpyr(&menuLists[0x0a] [1] [0],"Disabled");
	menuListVals[0x0a]=0;
	
	menuType[1] [5] = 0x0b02021a;
	strcpyr(&menuLists[0x0b] [0] [0],"ABC");
	strcpyr(&menuLists[0x0b] [1] [0],"ACB");
	menuListVals[0x0b]=0;
	
	menuType[1] [6] = 0x0c02021b;
	strcpyr(&menuLists[0x0c] [0] [0],"YM2149");
	strcpyr(&menuLists[0x0c] [1] [0],"AY8910");
	menuListVals[0x0c]=0;
	
	menuType[1] [7] = 0x0000043f;
	
	strcpyr(&menuItems[2] [0] [0],"HARDWARE");
	strcpyr(&menuItems[2] [1] [0],"Port #FE:");
	strcpyr(&menuItems[2] [2] [0],"Port #FF:");
	strcpyr(&menuItems[2] [3] [0],"ULA+:");
	strcpyr(&menuItems[2] [4] [0],"Snow Bug:");
	strcpyr(&menuItems[2] [5] [0],"Video Timings:");
	strcpyr(&menuItems[2] [6] [0],"Memory:");
	strcpyr(&menuItems[2] [7] [0],"MMC Mode:");
	strcpyr(&menuItems[2] [8] [0],"MMC Version:");	
	strcpyr(&menuItems[2] [9] [0],"END");
	
	menuType[2] [0] = 0x00000021;
	menuType[2] [1] = 0x0d020222;
	strcpyr(&menuLists[0x0d] [0] [0],"Issue 2");
	strcpyr(&menuLists[0x0d] [1] [0],"Issue 3");
	menuListVals[0x0d]=0;
	
	menuType[2] [2] = 0x0e020223;
	strcpyr(&menuLists[0x0e] [0] [0],"  Timex");
	strcpyr(&menuLists[0x0e] [1] [0],"SAA1099");
	menuListVals[0x0e]=0;
	
	menuType[2] [3] = 0x0f020224;
	strcpyr(&menuLists[0x0f] [0] [0]," Enabled");
	strcpyr(&menuLists[0x0f] [1] [0],"Disabled");
	menuListVals[0x0f]=0;
	
	menuType[2] [4] = 0x10020225;
	strcpyr(&menuLists[0x10] [0] [0],"Disabled");
	strcpyr(&menuLists[0x10] [1] [0]," Enabled");
	menuListVals[0x10]=0;
	
	menuType[2] [5] = 0x11030226;
	strcpyr(&menuLists[0x11] [0] [0],"  ULA-48");
	strcpyr(&menuLists[0x11] [1] [0]," ULA-128");
	strcpyr(&menuLists[0x11] [2] [0],"Pentagon");
	menuListVals[0x11]=0;
	
	menuType[2] [6] = 0x12050227;
	strcpyr(&menuLists[0x12] [0] [0],"Spectrum 128K/+2");
	strcpyr(&menuLists[0x12] [1] [0],"  Pentagon 1024K");
	strcpyr(&menuLists[0x12] [2] [0],"     Profi 1024K");
	strcpyr(&menuLists[0x12] [3] [0],"    Spectrum 48K");
	strcpyr(&menuLists[0x12] [4] [0]," Spectrum +2A/+3");
	menuListVals[0x12]=0;
	
	menuType[2] [7] = 0x13010228;
	strcpyr(&menuLists[0x13] [0] [0],"   Auto (VHD)");
	strcpyr(&menuLists[0x13] [1] [0],"SD Card 14MHz");
	strcpyr(&menuLists[0x13] [2] [0],"SD Card 28MHz");
	menuListVals[0x13]=0;
	
	menuType[2] [8] = 0x14030229;
	strcpyr(&menuLists[0x14] [0] [0],"DivMMC+ESXDOS");
	strcpyr(&menuLists[0x14] [1] [0],"       DivMMC");
	strcpyr(&menuLists[0x14] [2] [0],"        ZXMMC");
	menuListVals[0x14]=0;
	
	strcpyr(&menuItems[3] [0] [0],"RESET OPTIONS");
	strcpyr(&menuItems[3] [1] [0],"Warm Reset");
	strcpyr(&menuItems[3] [2] [0],"Cold Reset (Disk unload)");
	strcpyr(&menuItems[3] [3] [0],"Reset to ROM0 0Menu");
	strcpyr(&menuItems[3] [4] [0],"48K Basic Load (no lock)");
	strcpyr(&menuItems[3] [5] [0],"48K Basic Load (lock)");
	strcpyr(&menuItems[3] [6] [0],"Issue NMI");
	strcpyr(&menuItems[3] [7] [0],"Enter Multiface");	
	strcpyr(&menuItems[3] [8] [0],"END");
	
	menuType[3] [0] = 0x00000030;
	menuType[3] [1] = 0x00000631;
	menuType[3] [2] = 0x00000632;
	menuType[3] [3] = 0x00000633;
	menuType[3] [4] = 0x00000634;
	menuType[3] [5] = 0x00000635;
	menuType[3] [6] = 0x00000636;
	menuType[3] [7] = 0x00000637;
	
	strcpyr(&menuItems[4] [0] [0],"MODEL SELECT");
	strcpyr(&menuItems[4] [1] [0],"ZX Spectrum 48K");
	strcpyr(&menuItems[4] [2] [0],"ZX Spectrum 128K");
	strcpyr(&menuItems[4] [3] [0],"ZX Spectrum +3");
	strcpyr(&menuItems[4] [4] [0],"Pentagon 48K");
	strcpyr(&menuItems[4] [5] [0],"Pentagon 128K");
	strcpyr(&menuItems[4] [6] [0],"Pentagon 1024K");
	strcpyr(&menuItems[4] [7] [0],"END");
	
	menuType[4] [0] = 0x00000038;
	menuType[4] [1] = 0x00000739;
	menuType[4] [2] = 0x0000073a;
	menuType[4] [3] = 0x0000073b;
	menuType[4] [4] = 0x0000076c;
	menuType[4] [5] = 0x0000073d;
	menuType[4] [6] = 0x0000073e;
	
	strcpyr(&menuItems[5] [0] [0],"TAPE OPTIONS");
	strcpyr(&menuItems[5] [1] [0],"Pause/Continue");
	strcpyr(&menuItems[5] [2] [0],"Previous Part");
	strcpyr(&menuItems[5] [3] [0],"Next Part");
	strcpyr(&menuItems[5] [4] [0],"Fast Tape Load:");
	strcpyr(&menuItems[5] [5] [0],"Tape Sound:");
	strcpyr(&menuItems[5] [6] [0],"END");
	
	menuType[5] [0] = 0x00000041;
	menuType[5] [1] = 0x00000842;
	menuType[5] [2] = 0x00000843;
	menuType[5] [3] = 0x00000844;
	menuType[5] [4] = 0x00000445;
	menuType[5] [5] = 0x00000446;
	
	
	
	
}

int kbx,kby;
int kb_on;
unsigned char kb_matrix [8];

void drawKeyboard(int kx, int ky) {
	int inv;
	for (int y=0;y<5;y++) {
		for (int x=0;x<14;x++) {
			inv=0;
			if ((y==ky) && (x==kx)) inv=1;
			if ((y==3) && (x==0) && (cap_shift)) inv=1;
			if ((y==3) && (x==1) && (cap_shift)) inv=1;
			if ((y==3) && (x==12) && (cap_shift)) inv=1;
			if ((y==3) && (x==13) && (cap_shift)) inv=1;
			if ((y==4) && (x==0) && (sym_shift)) inv=1;
			if ((y==4) && (x==13) && (sym_shift)) inv=1;
			if (inv) writeChar(keyboard[y] [x],(y+2)*16*osd_char_width+x+3,1); else writeChar(keyboard[y] [x],(y+2)*16*osd_char_width+x+3,0);
		}
	}
};


void clearKeyboardMatrix() {
	for (int r=0;r<8;r++) {			//reset keyboard rows
		kb_matrix[r]=0xff;			
	}
	//if (cap_shift) kb_matrix[0]=0xfe;	//shift							
	//if (sym_shift) kb_matrix[7]=0xfd;	//shift							
}


int inputDelay=0;
int pause_on;
uint32_t joyclear;

const int delaysize=20;

void process_menu(uint32_t mask) {
	if (joyclear==0) return; //hacky - to stop continuous pressing
	
	if (mask & DPAD_UP) {
		if (cursorpos>1) {cursorpos--;} else {cursorpos=numitems;};
		updateCursor();			
		joyclear=0;	
	}
	
	if (mask & DPAD_DOWN) {
		if (cursorpos<numitems) {cursorpos++;} else {cursorpos=1;}
		updateCursor();			
		joyclear=0;	
	}
	
	int v;
	int mt;
	int m;
	int e;
	if (mask & FACE_A) {
		joyclear=0;	
		mt=((menuType[currmenu] [cursorpos] >> 8) & 0xff);		//get menutype
		switch (mt) {
			case 1:		//sub menu
				v=(menuType[currmenu] [cursorpos] >> 24) & 0xff; //get menu id
				clearOSD(0);	
				drawBorder();
				currmenu=v;
				numitems=writeMenu();
				cursorpos=1;
				oldpos=1;
				updateCursor();
			break;
			case 2:		//list				
				v=(menuType[currmenu] [cursorpos] >> 24) & 0xff; //get list id
				e=(menuType[currmenu] [cursorpos] >> 16) & 0xff; //get num lid entries
				m=menuListVals[v];
				m++;
				if (m==e) m=0;
				menuListVals[v]=m;
				writeMenu();
			break;
			case 3:		//yes no
				v=(menuType[currmenu] [cursorpos] >> 24) & 0xff; //get current state
				v=1-v;
				menuType[currmenu] [cursorpos]&=0x00ffffff;
				menuType[currmenu] [cursorpos]|=(v<<24);
				writeMenu();
			break;
			case 4:		//on off
				v=(menuType[currmenu] [cursorpos] >> 24) & 0xff; //get current state
				v=1-v;
				menuType[currmenu] [cursorpos]&=0x00ffffff;
				menuType[currmenu] [cursorpos]|=(v<<24);
				writeMenu();
			break;
			case 5:		//map joystick
				mapJoystick();
				clearOSD(0);
				drawBorder();
				writeMenu();			
				updateCursor();
			break;
			//mod bits
			//0 = RShift
			//1 = Alt
			//2 = Ctrl
			//0x200 = F10
			//0x400 = F11
			case 6:		//reset fkey presses				
				switch ((menuType[currmenu] [cursorpos]) & 0xff) {
					case 0x31:
						IO_RW(SEND_FKEYS)=0x40400;
						menu_on=0;
					break;
					case 0x32:
						IO_RW(SEND_FKEYS)=0x20400;
						menu_on=0;
					break;
					case 0x33:
						IO_RW(SEND_FKEYS)=0x60400;
						menu_on=0;
					break;
					case 0x34:
						IO_RW(SEND_FKEYS)=0x00200;
						key_auto=1;
						auto_kptr=0;
						kb_on=0;
						menu_on=0;
						auto_delay=AUTO_KEY_DELAY;
						clearKeyboardMatrix();
						cap_shift=0;
						sym_shift=0;
					break;
					case 0x35:
						IO_RW(SEND_FKEYS)=0x10200;
						key_auto=1;
						auto_kptr=0;
						kb_on=0;
						menu_on=0;
						auto_delay=AUTO_KEY_DELAY;
						clearKeyboardMatrix();
						cap_shift=0;
						sym_shift=0;
					break;
					case 0x36:
						IO_RW(SEND_FKEYS)=0x00400;
						menu_on=0;
					break;
					case 0x37:
						IO_RW(SEND_FKEYS)=0x10400;
						menu_on=0;
					break;
				}
				menu_on=0;
				IO_RW(IO_MENU_ON)=menu_on;
				IO_RW(IO_KB_ON)=key_auto<<1 | kb_on;
			break;
			case 7:		//model change fkey presses				
				switch ((menuType[currmenu] [cursorpos]) & 0xff) {
					case 0x39:
						IO_RW(SEND_FKEYS)=0x20001;
					break;
					case 0x3a:
						IO_RW(SEND_FKEYS)=0x20002;
					break;
					case 0x3b:
						IO_RW(SEND_FKEYS)=0x20004;
					break;
					case 0x3c:
						IO_RW(SEND_FKEYS)=0x20008;
					break;
					case 0x3d:
						IO_RW(SEND_FKEYS)=0x20010;
					break;
					case 0x3e:
						IO_RW(SEND_FKEYS)=0x20020;						
					break;									
				}
				menu_on=0;
				IO_RW(IO_MENU_ON)=menu_on;
			break;
			case 8:		//tape options fkey presses				
				switch ((menuType[currmenu] [cursorpos]) & 0xff) {
					case 0x42:
						IO_RW(SEND_FKEYS)=0x00001;
					break;
					case 0x43:
						IO_RW(SEND_FKEYS)=0x00002;
					break;
					case 0x44:
						IO_RW(SEND_FKEYS)=0x00004;
					break;					
				}
				menu_on=0;
				IO_RW(IO_MENU_ON)=menu_on;
			break;
		}
		encode_status();
		
		if (mt==9) status0 |= 1;	//set reset flag
		
		IO_RW(IO_STATUS1)=status1;
		IO_RW(IO_STATUS0)=status0;
		
		
		if (mt==9) {
			status0 &= 0xfffffffe;	//clear reset flag
			IO_RW(IO_STATUS0)=status0;
		}
		
	}
	if (mask & FACE_B) {
		joyclear=0;	
		if (currmenu>0) {
			clearOSD(0);	
			drawBorder();
			currmenu=0;
			numitems=writeMenu();
			cursorpos=1;
			oldpos=1;
			updateCursor();
		}
	}
	
}






void setKeyboardModifier(int kx,int ky) {
			switch (ky) {		//set bitmask based on key												
				case 3:			//cap shift extend z x c v b n m up cap shift
					switch (kx) {
						case 0:		//caps shift
						case 1:
							cap_shift=1-cap_shift;
						break;						
						case 12:	//cap shift
						case 13:					
							cap_shift=1-cap_shift;
						break;						
					}
				break;
				case 4:			//sym shift ; " , . space left down right sym shift
					switch (kx) {
						case 0:		//sym shift
							sym_shift=1-sym_shift;
						break;						
						case 13: //sym shift
							sym_shift=1-sym_shift;
						break;						
					}
				break;
			}
			
}


void setKeyboardMatrix(int kx,int ky) {
			switch (ky) {		//set bitmask based on key
				case 0:			//break edit 1 2 3 4 5 6 7 8 9 0 delete
					switch (kx) {
						case 0:		//break
							kb_matrix[0]=0xfe;	//shift
							kb_matrix[7]=0xfe;	//space
						break;
						case 1:		//edit
							kb_matrix[0]=0xfe;	//shift
							kb_matrix[3]=0xfe;	//1
						break;
						case 2:		//1							
							kb_matrix[3]=0xfe;	//1
						break;
						case 3:		//2							
							kb_matrix[3]=0xfd;	//2
						break;
						case 4:		//3							
							kb_matrix[3]=0xfb;	//3
						break;
						case 5:		//4							
							kb_matrix[3]=0xf7;	//4
						break;
						case 6:		//5							
							kb_matrix[3]=0xef;	//5
						break;
						case 7:		//6							
							kb_matrix[4]=0xef;	//6
						break;
						case 8:		//7							
							kb_matrix[4]=0xf7;	//7
						break;
						case 9:		//8							
							kb_matrix[4]=0xfb;	//8
						break;
						case 10:		//9							
							kb_matrix[4]=0xfd;	//9
						break;
						case 11:		//0							
							kb_matrix[4]=0xfe;	//0
						break;
						case 12:		//delete							
						case 13:
							kb_matrix[0]=0xfe;	//shift
							kb_matrix[4]=0xfe;	//0
						break;						
					}
				break;
				case 1:			//true vid inv vid q w e r t y u i o p
					switch (kx) {
						case 0:		//true video
						case 1:
							kb_matrix[0]=0xfe;	//shift
							kb_matrix[3]=0xfb;	//3
						break;
						case 2:		//inv video
						case 3:
							kb_matrix[0]=0xfe;	//shift
							kb_matrix[3]=0xf7;	//4
						break;
						case 4:		//Q							
							kb_matrix[2]=0xfe;	//Q
						break;
						case 5:		//W							
							kb_matrix[2]=0xfd;	//W
						break;
						case 6:		//E							
							kb_matrix[2]=0xfb;	//E
						break;
						case 7:		//R							
							kb_matrix[2]=0xf7;	//R
						break;
						case 8:		//T							
							kb_matrix[2]=0xef;	//T
						break;
						case 9:		//Y							
							kb_matrix[5]=0xef;	//Y
						break;
						case 10:	//U							
							kb_matrix[5]=0xf7;	//U
						break;
						case 11:	//I							
							kb_matrix[5]=0xfb;	//I
						break;
						case 12:	//O							
							kb_matrix[5]=0xfd;	//O
						break;
						case 13:	//P							
							kb_matrix[5]=0xfe;	//P
						break;						
					}
				break;
				case 2:			//cap lock graph a s d f g h j k l enter
					switch (kx) {
						case 0:		//caps lock
						case 1:
							kb_matrix[0]=0xfe;	//shift
							kb_matrix[3]=0xfd;	//2
						break;						
						case 2:  //graph
							kb_matrix[0]=0xfe;	//shift
							kb_matrix[4]=0xfd;	//9
						break;
						case 3:		//A							
							kb_matrix[1]=0xfe;	//A
						break;
						case 4:		//S							
							kb_matrix[1]=0xfd;	//S
						break;
						case 5:		//D							
							kb_matrix[1]=0xfb;	//D
						break;
						case 6:		//F							
							kb_matrix[1]=0xf7;	//F
						break;
						case 7:		//G							
							kb_matrix[1]=0xef;	//G
						break;
						case 8:		//H						
							kb_matrix[6]=0xef;	//H
						break;
						case 9:		//J							
							kb_matrix[6]=0xf7;	//J
						break;
						case 10:	//K							
							kb_matrix[6]=0xfb;	//K
						break;
						case 11:	//L							
							kb_matrix[6]=0xfd;	//L
						break;
						case 12:	//enter							
						case 13:					
							kb_matrix[6]=0xfe;	//ENTER
						break;						
					}
				break;
				case 3:			//cap shift extend z x c v b n m up cap shift
					switch (kx) {
						case 0:		//caps shift
						case 1:
							kb_matrix[0]=0xfe;	//shift							
						break;						
						case 2:  //extend mode
						case 3:
							kb_matrix[0]=0xfe;	//shift
							kb_matrix[7]=0xfd;	//sym shift
						break;
						case 4:		//Z							
							kb_matrix[0]=0xfd;	//Z
						break;
						case 5:		//X							
							kb_matrix[0]=0xfb;	//X
						break;
						case 6:		//C							
							kb_matrix[0]=0xf7;	//C
						break;
						case 7:		//V							
							kb_matrix[0]=0xef;	//V
						break;
						case 8:		//B							
							kb_matrix[7]=0xef;	//B
						break;
						case 9:		//N						
							kb_matrix[7]=0xf7;	//N
						break;
						case 10:		//M							
							kb_matrix[7]=0xfb;	//M
						break;
						case 11:	//UP							
							kb_matrix[0]=0xfe;	//shift
							kb_matrix[4]=0xf7;	//7
						break;									
						case 12:	//cap shift
						case 13:					
							kb_matrix[0]=0xfe;	//shift							
						break;						
					}
				break;
				case 4:			//sym shift ; " , . space left down right sym shift
					switch (kx) {
						case 0:		//sym shift
							kb_matrix[7]=0xfd;	//sym shift
						break;
						case 1:		//;
							kb_matrix[7]=0xfd;	//sym shift
							kb_matrix[5]=0xfd;	//O
						break;
						case 2:		//"							
							kb_matrix[7]=0xfd;	//sym shift
							kb_matrix[5]=0xfe;	//P
						break;
						case 3:		//,							
							kb_matrix[7]=0xf5;	//sym shift & N							
						break;
						case 4:		//.							
							kb_matrix[7]=0xf9;	//sym shift & M							
						break;
						case 5:		//space											
						case 6:		//														
						case 7:		//														
						case 8:		//														
						case 9:		//							
							kb_matrix[7]=0xfe;	//space
						break;
						case 10:		//left							
							kb_matrix[0]=0xfe;	//shift
							kb_matrix[3]=0xef;	//5
						break;
						case 11:		//down
							kb_matrix[0]=0xfe;	//shift
							kb_matrix[4]=0xef;	//6
						break;
						case 12:		//right
							kb_matrix[0]=0xfe;	//shift
							kb_matrix[4]=0xfb;	//8
						break;
						case 13: //sym shift
							kb_matrix[7]=0xfd;	//sym shift
						break;						
					}
				break;
			}			
			if (kb_on) {
				if (cap_shift) kb_matrix[0]&=0xfe;	//shift							
				if (sym_shift) kb_matrix[7]&=0xfd;	//shift							
			}
			
}

//int nmi=0;
void processInput()
{
	uint32_t mask=readJoypad();		
	if (mask==0) joyclear=1; 	//debounce
	int kb_press;
	int mod_press;
	
	
	if (inputDelay) inputDelay--;
	
	//if (nmi) nmi--;
	//if (nmi==0) {
		 IO_RW(SEND_FKEYS)=0x0;
	//}
		
	
	
	if ((mask & L_TRIG) && (mask & START) && (joyclear==1))	{
		pause_on=1-pause_on; 
		IO_RW(PAUSE_Z80)=pause_on;
		joyclear=0;		
	}
	
	//mod bits
	//0 = RShift
	//1 = Alt
	//2 = Ctrl
	
	if ((mask & L_TRIG) && (mask & SELECT) && (joyclear==1))	{				
		//nmi=32;
		joyclear=0;			
		IO_RW(SEND_FKEYS)=0x00400;		
	}
	
	
	
	//if ((mask & START) && (!(mask & L_TRIG)) && (joyclear==1))	{
	if ((mask & START) && (joyclear==1))	{
		if (menu_on) {
			menu_on=0;
			IO_RW(IO_MENU_ON)=0;
		}
		else {
			kb_on=0; 
			IO_RW(IO_KB_ON)=0;			
			clearOSD(0);
			drawBorder();
			numitems=writeMenu();			
			updateCursor();
			menu_on=1;
			IO_RW(IO_MENU_ON)=1;
		}
		joyclear=0;
	}
	
	if ((mask & SELECT) && (joyclear==1))	{
		if (kb_on) {
			kb_on=0;
			IO_RW(IO_KB_ON)=0;
			//clearOSD(0);
		}
		else {
			menu_on=0; 
			IO_RW(IO_MENU_ON)=0;	
			clearOSD(0);
			drawBorder();
			drawKeyboard(kbx,kby);
			kb_on=1;
			IO_RW(IO_KB_ON)=1;
		}
		joyclear=0;
	}
	
	//if ((kb_on) || (key_auto)) {		
	if (kb_on | key_auto) {		
		if (inputDelay==0) {
		if (mask) inputDelay=delaysize;
		
		if (mask & DPAD_UP) {
			if (kby>0) {kby--;} else {kby=4;}
		}
	
		if (mask & DPAD_DOWN) {
			if (kby<4) {kby++;} else {kby=0;}
		}
		if (mask & DPAD_LEFT) {
			if (kbx>0) {kbx--;} else {kbx=13;}
		}
	
		if (mask & DPAD_RIGHT) {
			if (kbx<13) {kbx++;} else {kbx=0;}
		}
		
		if (mask & FACE_A) {kb_press=1;} else {kb_press=0;}
		if (mask & FACE_X) {mod_press=1;} else {mod_press=0;}
		
		if (kb_on) clearKeyboardMatrix();
		
		/*keyboard_matrix is
		row 0 - SHIFT Z X C V
		row 1 - A S D F G
		row 2 - Q W E R T
		row 3 - 1 2 3 4 5
		row 4 - 0 9 8 7 6
		row 5 - P O I U Y
		row 6 - ENTER L K J H
		row 7 - SPACE SYM M N B
		*/
		
		//if (kb_press) setKeyboardMatrix(kbx,kby);
		//if (mod_press) setKeyboardModifier(kbx,kby);
		if (key_auto) {
			if (auto_delay) {
				auto_delay--;
			}
			else
			{
				if (auto_key[auto_kptr] [0]<255) {setKeyboardMatrix(auto_key[auto_kptr] [0],auto_key[auto_kptr] [1]);} else {clearKeyboardMatrix();}
				auto_kptr++;
				auto_delay=AUTO_KEY_REPEAT;
				if (auto_kptr==AUTO_KEY_MAX) {
					//auto_kptr=0;
					key_auto=0;
					IO_RW(IO_KB_ON)=kb_on;
				}
			}
		}
		else {
			if (kb_press) setKeyboardMatrix(kbx,kby);
			if (mod_press) setKeyboardModifier(kbx,kby);
		}

		IO_RW(IO_KEYB_0)=kb_matrix[0];
		IO_RW(IO_KEYB_1)=kb_matrix[1];
		IO_RW(IO_KEYB_2)=kb_matrix[2];
		IO_RW(IO_KEYB_3)=kb_matrix[3];
		IO_RW(IO_KEYB_4)=kb_matrix[4];
		IO_RW(IO_KEYB_5)=kb_matrix[5];
		IO_RW(IO_KEYB_6)=kb_matrix[6];
		IO_RW(IO_KEYB_7)=kb_matrix[7];
		}
				
	if (kb_on) drawKeyboard(kbx,kby);
	}
	
	
	if (menu_on) process_menu(mask);

		
	
}

int kbmx,kbmy,kbm_num;
//uint32_t keyjoy[10];		//31-24 reserved 23-8 - address row - 7-0 key bitmask

int keyjoy_xloc[10]; //store to location to allow easy printing of values
int keyjoy_yloc[10];
int keyjoy_locw[10];


void writeCurrentKey(int key,uint32_t address){
	for (int i=0;i<keyjoy_locw[key];i++) {
		writeChar(keyboard[keyjoy_yloc[key]] [keyjoy_xloc[key]+i],address+i,0);
	}
}



void mapJoystick() {
	kbmx=0;
	kbmy=0;
	kbm_num=0;
	uint32_t mask;
	
	mask=readJoypad();				
	while ((mask & FACE_A)==FACE_A) {
		mask=readJoypad();				
	}
	
	while (kbm_num<10) {
	clearOSD(0);
	
	
	
	
	
	
	//write currently mapped values
	writeString("   UP=",144,24);
	writeCurrentKey(0,24*osd_char_width+24);
	writeString(" DOWN=",144,32);
	writeCurrentKey(1,32*osd_char_width+24);
	writeString(" LEFT=",144,40);
	writeCurrentKey(2,40*osd_char_width+24);
	writeString("RIGHT=",144,48);
	writeCurrentKey(3,48*osd_char_width+24);
	writeString("    A=",144,56);
	writeCurrentKey(4,56*osd_char_width+24);
	writeString("    B=",144,64);
	writeCurrentKey(5,64*osd_char_width+24);
	writeString("    X=",144,72);
	writeCurrentKey(6,72*osd_char_width+24);
	writeString("    Y=",144,80);
	writeCurrentKey(7,80*osd_char_width+24);
	writeString("    L=",144,88);
	writeCurrentKey(8,88*osd_char_width+24);
	writeString("    R=",144,96);
	writeCurrentKey(9,96*osd_char_width+24);
	
	
	
	writeString ("Map Key to Pocket Button ",8,8);
	switch (kbm_num) {
		case 0: writeString("UP",208,8);
		break;
		case 1: writeString("DOWN",208,8);	
		break;
		case 2: writeString("LEFT",208,8);			
		break;
		case 3: writeString("RIGHT",208,8);
		break;
		case 4: writeString("A",208,8);
		break;
		case 5: writeString("B",208,8);	
		break;
		case 6: writeString("X",208,8);			
		break;
		case 7: writeString("Y",208,8);
		break;
		case 8: writeString("L",208,8);
		break;
		case 9: writeString("R",208,8);
		break;
	}
	
	
	
	mask=0;
	while ((mask & FACE_A)!=FACE_A) {
		mask=readJoypad();				
		drawKeyboard(kbmx,kbmy);
		if (inputDelay) inputDelay--;
		if (inputDelay==0) {			
			if (mask) inputDelay=delaysize;
		
			if (mask & DPAD_UP) {
				if (kbmy>0) {kbmy--;} else {kbmy=4;}
			}
	
			if (mask & DPAD_DOWN) {
				if (kbmy<4) {kbmy++;} else {kbmy=0;}
			}
			if (mask & DPAD_LEFT) {
				if (kbmx>0) {kbmx--;} else {kbmx=13;}
			}
	
			if (mask & DPAD_RIGHT) {
				if (kbmx<13) {kbmx++;} else {kbmx=0;}
			}
		}
	}
	while ((mask & FACE_A)==FACE_A) {
		mask=readJoypad();				
	}

	keyjoy_xloc[kbm_num]=kbmx;
	keyjoy_yloc[kbm_num]=kbmy;
	keyjoy_locw[kbm_num]=1;
	if ((kbmx==0) && (kbmy<4)) keyjoy_locw[kbm_num]=2;
	if ((kbmx==1) && (kbmy<4)) {
			keyjoy_locw[kbm_num]=2;
			keyjoy_xloc[kbm_num]=kbmx-1;
	}
	if ((kbmx==2) && (kbmy==1)) keyjoy_locw[kbm_num]=2;
	if ((kbmx==3) && (kbmy==1)) {
		keyjoy_locw[kbm_num]=2;
		keyjoy_xloc[kbm_num]=kbmx-1;
	}
	if ((kbmx==2) && (kbmy==3)) keyjoy_locw[kbm_num]=2;
	if ((kbmx==3) && (kbmy==3)) {
		keyjoy_locw[kbm_num]=2;
		keyjoy_xloc[kbm_num]=kbmx-1;
	}
	if ((kbmx==12) && (kbmy==0)) keyjoy_locw[kbm_num]=2;
	if ((kbmx==13) && (kbmy==0)) {
		keyjoy_locw[kbm_num]=2;
		keyjoy_xloc[kbm_num]=kbmx-1;
	}
	if ((kbmx==12) && (kbmy==2)) keyjoy_locw[kbm_num]=2;
	if ((kbmx==13) && (kbmy==2)) {
		keyjoy_locw[kbm_num]=2;
		keyjoy_xloc[kbm_num]=kbmx-1;
	}if ((kbmx==12) && (kbmy==3)) keyjoy_locw[kbm_num]=2;
	if ((kbmx==13) && (kbmy==3)) {
		keyjoy_locw[kbm_num]=2;
		keyjoy_xloc[kbm_num]=kbmx-1;
	}	
	if ((kbmx>5) && (kbmx<=9) && (kbmy==4)) {
		keyjoy_locw[kbm_num]=5;
		keyjoy_xloc[kbm_num]=5;
	}
	
	clearKeyboardMatrix();
	setKeyboardMatrix(kbmx,kbmy);

	IO_RW(IO_JOYK_0_0+(0x20*kbm_num))=kb_matrix[0];
	IO_RW(IO_JOYK_0_1+(0x20*kbm_num))=kb_matrix[1];
	IO_RW(IO_JOYK_0_2+(0x20*kbm_num))=kb_matrix[2];
	IO_RW(IO_JOYK_0_3+(0x20*kbm_num))=kb_matrix[3];
	IO_RW(IO_JOYK_0_4+(0x20*kbm_num))=kb_matrix[4];
	IO_RW(IO_JOYK_0_5+(0x20*kbm_num))=kb_matrix[5];
	IO_RW(IO_JOYK_0_6+(0x20*kbm_num))=kb_matrix[6];
	IO_RW(IO_JOYK_0_7+(0x20*kbm_num))=kb_matrix[7];
		
	kbm_num++;
	}
					
}


uint32_t dataslot_id;
uint32_t dataslot_size_u;
uint32_t dataslot_size_l;
//uint64_t dataslot_size;


int imageDSKMounted;
int imageVHDMounted;
uint32_t disk_type;		
//uint32_t disk_size;
uint32_t disk_rw;

void readSector(unsigned int address, unsigned int size,unsigned int type) {	
//	if (size>512) return;	//max supported sector size is 512 bytes
	//if (address+size > disk_size) return;	//sector request beyond disk size limit
	
	IOCTL_RW(COMPLETE_ACK)=0;
	IOCTL_RW(SET_SIZE)=size;
	if (address>=0x80000) {
		IOCTL_RW(SET_ADDR_L)=(address-0x80000)*0x200;
		IOCTL_RW(SET_ADDR_U)=(address >> 23);
	} else {
		IOCTL_RW(SET_ADDR_L)=address*0x200;
		IOCTL_RW(SET_ADDR_U)=0x0;
	}
	
	IOCTL_RW(SET_ID)=type;
	IOCTL_RW(DOWNLOAD)=1;
	
	while (IOCTL_RW(COMPLETE_ACK)==0) {	//wait until download finishes		
	}
	IOCTL_RW(DOWNLOAD)=0;	
	IOCTL_RW(COMPLETE_ACK)=1;
}

void mountDSK() {

//	if ((dataslot_size_l<0x100) && (dataslot_size_u==0)) return;		//file too small
	
	imageDSKMounted=1;
	disk_rw=1;
	//disk_size=dataslot_size_l;
	disk_type=dataslot_id;
	

	IOCTL_RW(DISK_SIZE_LOW)=dataslot_size_l;
	IOCTL_RW(DISK_SIZE_HIGH)=dataslot_size_u;
	IOCTL_RW(DISK_MOUNTED)=((disk_rw<<1) & 2) + (imageDSKMounted & 1);
	IOCTL_RW(DISK_MOUNTED)=0;		//Mount signal is just pulsed, not continually asserted
		
}

void mountVHD() {

//	if ((dataslot_size_l<0x100) && (dataslot_size_u==0)) return;		//file too small

	imageVHDMounted=1;
	disk_rw=1;
	//disk_size=dataslot_size_l;
	disk_type=dataslot_id;
	
	IOCTL_RW(DISK_SIZE_LOW)=dataslot_size_l;
	IOCTL_RW(DISK_SIZE_HIGH)=dataslot_size_u;
	IOCTL_RW(VHD_MOUNTED)=((disk_rw<<1) & 2) + (imageVHDMounted & 1);
	IOCTL_RW(VHD_MOUNTED)=0;		//Mount signal is just pulsed, not continually asserted
}


void set_initial_keyval (unsigned int id, unsigned int addr,unsigned int x,unsigned int y,unsigned int w) {
	keyjoy_xloc[id]=x;
	keyjoy_yloc[id]=y;
	keyjoy_locw[id]=w;	
	clearKeyboardMatrix();
	setKeyboardMatrix(x,y);
	IO_RW(addr)=kb_matrix[0];
	IO_RW(addr+4)=kb_matrix[1];
	IO_RW(addr+8)=kb_matrix[2];
	IO_RW(addr+12)=kb_matrix[3];
	IO_RW(addr+16)=kb_matrix[4];
	IO_RW(addr+20)=kb_matrix[5];
	IO_RW(addr+24)=kb_matrix[6];
	IO_RW(addr+28)=kb_matrix[7];
}


//Note - external IO is 32 bit
int main(void)
{
	debug_on=0;
	filename[0]=0;
	ext[0]=0;
	
	initMenus();	
	encode_status();
	//hold core in init_reset	
	IO_RW(IO_STATUS1)=status1;
	status0 |= 1;	//set reset flag
	IO_RW(IO_STATUS0)=status0;		
	IOCTL_RW(ROM_LOADED)=0;
	
	pause_on=0;
	setOSDSize(256,128);
	clearOSD(0);
	
	drawBorder();
	
	
	
	currmenu=0;
	numitems=writeMenu();
	cursorpos=1;
	oldpos=1;
	updateCursor();
	menu_on=0;
	IO_RW(IO_MENU_ON)=menu_on;
	
	kbx=0;
	kby=0;
	
	
	//all this stuff below is rather manual just to set up
	//starting keys for the May Mapped Joystick

		/*keyboard_matrix is
		row 0 - SHIFT Z X C V		FEFE
		row 1 - A S D F G			FDFE
		row 2 - Q W E R T			FBFE
		row 3 - 1 2 3 4 5			F7FE
		row 4 - 0 9 8 7 6			EFFE
		row 5 - P O I U Y			DFFE
		row 6 - ENTER L K J H		BFFE
		row 7 - SPACE SYM M N B		7FFE
		*/
	
	set_initial_keyval(0,IO_JOYK_0_0,4,1,1);	// Q (UP)
	set_initial_keyval(1,IO_JOYK_1_0,3,2,1);	// A (DOWN)
	set_initial_keyval(2,IO_JOYK_2_0,12,1,1);	// O (LEFT)
	set_initial_keyval(3,IO_JOYK_3_0,13,1,1);	// P (RIGHT)
	set_initial_keyval(4,IO_JOYK_4_0,5,4,5);	// SPACE (A)
	set_initial_keyval(5,IO_JOYK_5_0,13,2,2);	// ENTER (B)
	set_initial_keyval(6,IO_JOYK_6_0,11,0,1);	// 0 (X)
	set_initial_keyval(7,IO_JOYK_7_0,0,4,1);	// Sym Shift (Y)
	set_initial_keyval(8,IO_JOYK_8_0,0,3,2);	// Cap Shift (L)
	set_initial_keyval(9,IO_JOYK_9_0,11,1,1);	// I (R)
		
	
	
	kb_on=0;
	IO_RW(IO_KB_ON)=kb_on;
	IO_RW(SEND_FKEYS)=0x0;
	
	imageDSKMounted=0;
	IOCTL_RW(DISK_MOUNTED)=0;
	imageVHDMounted=0;
	IOCTL_RW(VHD_MOUNTED)=0;
	
	//int y=0;
	
	//load the ROM into memory
	IOCTL_RW(COMPLETE_ACK)=0;
	IOCTL_RW(SET_SIZE)=0x34000;
	IOCTL_RW(SET_ADDR_U)=0x0;
	IOCTL_RW(SET_ADDR_L)=0x0;
	IOCTL_RW(SET_ID)=0x200;
	IOCTL_RW(DOWNLOAD)=1;
	
	while (IOCTL_RW(COMPLETE_ACK)==0) {	//wait until download finishes		
	}
	IOCTL_RW(DOWNLOAD)=0;
	IOCTL_RW(ROM_LOADED)=1;
	IOCTL_RW(COMPLETE_ACK)=1;
	
	//bring core out of init_reset	
	status0 &= 0xfffffffe;	//clear reset flag
	IO_RW(IO_STATUS0)=status0;			
	
	
	while(1)
	{	
		if (IOCTL_RW(REQ_ACK)==1) {		//Core has loaded a dataslot
			
			IOCTL_RW(REQ_ACK)=1;		//Acknowledge dataslot change
			IOCTL_RW(COMPLETE_ACK)=0;		//reset download complete ackknowledge
			dataslot_id=IOCTL_RW(SET_ID);			//read in datalot_id
			dataslot_size_u=IOCTL_RW(SET_ADDR_U);		//read in top 16 bits of file size
			dataslot_size_l=IOCTL_RW(SET_ADDR_L);		//read in lower 32 bits of file size
			//dataslot_size=((uint_64t)dataslot_size_u<<32) | dataslot_size_l;
			
			//first we need to load in the filename
			//so we can access cfg files (not for VHD images though)
			if (dataslot_id!=3) {
				IOCTL_RW(SET_ADDR_L)=0xFFFFFFFF;			//address FFFFFFFF signals filename load
				IOCTL_RW(SET_ADDR_U)=0x0;			//address FFFFFFFF signals filename load
				IOCTL_RW(SET_ID)=dataslot_id;
				IOCTL_RW(DOWNLOAD)=1;	
	
				while (IOCTL_RW(COMPLETE_ACK)==0) {	//wait until download finishes			
				}
	
				IOCTL_RW(DOWNLOAD)=0;	//Turn off download request
				IOCTL_RW(COMPLETE_ACK)=1;	//Acknowledge load complete
				
				unsigned int data;
				int i;
				for (i=0;i<64;i++) {
					IOCTL_RW(FNAME_ADDR)=i;
					data=IOCTL_RW(FNAME_DATA);
					filename[i*4]=(char) data & 0xff;
					filename[i*4+1]=(char) (data >> 8) & 0xff;
					filename[i*4+2]=(char) (data >> 16) & 0xff;
					filename[i*4+3]=(char) (data >> 24) & 0xff;				
				}
				
				i=strlen(filename);
				if (i>4) {				//*.xxx  to have a valid extension filename length must be greater than 4 chars
					ext[0]=make_upper(filename[i-3]);
					ext[1]=make_upper(filename[i-2]);
					ext[2]=make_upper(filename[i-1]);
					ext[3]=0;
					filename[i-3]='c';		//now replace the extension with 'cfg'
					filename[i-2]='f';
					filename[i-1]='g';
				}
				
				/*for (i=0;i<64;i++) {		//update memory with path to config file
					IOCTL_RW(FNAME_ADDR)=i;					
					data=filename[i*4] & 0xff;
					data|=(filename[i*4+1] << 8) & 0xff;
					data|=(filename[i*4+2] << 16) & 0xff;
					data|=(filename[i*4+3] << 24) & 0xff;					
					IOCTL_RW(FNAME_DATA)=data;
					IOCTL_RW(FNAME_WRITE)=1;
					IOCTL_RW(FNAME_WRITE)=0;
				}
				IOCTL_RW(FNAME_ADDR)=64;		//create if doesn't exist
				IOCTL_RW(FNAME_DATA)=1;
				IOCTL_RW(FNAME_WRITE)=1;
				IOCTL_RW(FNAME_WRITE)=0;
				
				IOCTL_RW(SET_ADDR_L)=0xFFFFFFFE;			//address FFFFFFFF signals filename load
				IOCTL_RW(SET_ADDR_U)=0x0;			//address FFFFFFFF signals filename load
				IOCTL_RW(SET_ID)=0x400;
				IOCTL_RW(SET_SIZE)=32;	
				IOCTL_RW(UPLOAD)=1;	
				
	
				while (IOCTL_RW(COMPLETE_ACK)==0) {	//wait until download finishes			
				}
	
				IOCTL_RW(UPLOAD)=0;	//Turn off download request
				IOCTL_RW(COMPLETE_ACK)=1;	//Acknowledge load complete
				*/
			
			}
			
			
			
			
			IOCTL_RW(COMPLETE_ACK)=0;		//reset download complete ackknowledge			
			switch (dataslot_id) {
				case 1:			//Disk image
					if (strcmp(ext,"TRD")==0) ioctl_index=0x00;
					if (strcmp(ext,"IMG")==0) ioctl_index=0x40;
					if (strcmp(ext,"DSK")==0) ioctl_index=0x80;
					if (strcmp(ext,"MGT")==0) ioctl_index=0xC0;					
					ioctl_index+=dataslot_id;
					IOCTL_RW(SET_INDEX)=ioctl_index;
					mountDSK();
				break;
				case 3:			//VHD image					
					ioctl_index=dataslot_id;
					IOCTL_RW (SET_INDEX)=ioctl_index;
					mountVHD();
				break;
				default:   //tape/snapshots				
				//load the file into memory
					if (strcmp(ext,"TAP")==0) ioctl_index=0x00;
					if (strcmp(ext,"CSW")==0) ioctl_index=0x40;
					if (strcmp(ext,"TZX")==0) ioctl_index=0x80;
					if (strcmp(ext,"Z80")==0) ioctl_index=0x00;
					if (strcmp(ext,"SNA")==0) ioctl_index=0x40;
					if (strcmp(ext,"VHD")==0) ioctl_index=0x00;										
					ioctl_index+=dataslot_id;
					IOCTL_RW(SET_INDEX)=ioctl_index;
					IOCTL_RW(SET_SIZE)=dataslot_size_l;
					IOCTL_RW(SET_ADDR_U)=0x0;
					IOCTL_RW(SET_ADDR_L)=0x0;
					IOCTL_RW(SET_ID)=dataslot_id;
					IOCTL_RW(DOWNLOAD)=1;	
					while (IOCTL_RW(COMPLETE_ACK)==0) {	//wait until download finishes		
					}
					IOCTL_RW(DOWNLOAD)=0;	//Turn off download request
					IOCTL_RW(COMPLETE_ACK)=1;	//Acknowledge load complete
				break;
				
			}
			IOCTL_RW(REQ_ACK)=0;	//turn off new dataslot acknowledge
			
			
			
		}		
		
		if ((IOCTL_RW(DISK_BUFF_RD) & 1) && (imageDSKMounted)) {		//core has requested disk sector
			readSector(IOCTL_RW(DISK_BUFF_ADDR),IOCTL_RW(DISK_SIZE_LOW),1);
		}
		if ((IOCTL_RW(DISK_BUFF_RD) & 2) && (imageVHDMounted)) {		//core has requested vhd sector
			readSector(IOCTL_RW(VHD_BUFF_ADDR),IOCTL_RW(DISK_SIZE_LOW),3);
		}

		processInput();
		
		//high 32 bits - status [63:32]
		//low 32 bit - MMC Ver, Vert Crop, Scale, Snow Bug,
		//SPEED REQ, (24:22) CPU Speed
		//GS, Joystick, Scandoubler, ULA+, Port #FF (21:13)
		//ARCH (12:8) OR SNAP_HW (12:8) Status[12:8] - 12:10 - memory - 9:8 - Video Timings
		//7:0
		if (IO_RW(IO_STATUSC)) {		//Core has requested status change
			status0=IO_RW(IO_STATUS0);
			status1=IO_RW(IO_STATUS1);
			IO_RW(IO_STATUSC)=1;		//acknowledge status change request
			//need to decode to update menus
			decode_status();
			//then write back to core
			IO_RW(IO_STATUS1)=status1;
			IO_RW(IO_STATUS0)=status0;			
		}
			

	}
	return(0);
}
