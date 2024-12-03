

	/*"Spectrum;;",
	"S0,TRDIMGDSKMGT,Load Disk;",
	"F2,TAPCSWTZX,Load Tape;",
	"F4,Z80SNA,Load Snapshot;",
	"S1,VHD,Load DivMMC;",
	"-;",

	"P1,Audio & Video;",
	"P1-;",
	"P1O[5:4],Aspect Ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O[16:15],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%;",
	"P1-;",
	"P1O[38],Narrow Border,No,Yes;",
	"H2d1P1O[28],Vertical Crop,No,Yes;",
	"h2d1P1O[29:28],Vertical Crop,No,270,216;",
	"P1O[27:26],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"P1-;",
	"P1O[21:20,General Sound,512KB,1MB,2MB,Disabled;",
	"P1O[3:2],Stereo Mix,none,25%,50%,100%;",
	"P1-;",
	"P1O[39],PSG/FM,Enabled,Disabled;",
	"P1O[40],PSG Stereo,ABC,ACB;",
	"P1O[41],PSG Model,YM2149,AY8910;",

	"P2,Hardware;",
	"P2-;",
	"P2O[7],Port #FE,Issue 2,Issue 3;",
	"P2O[13],Port #FF,Timex,SAA1099;",
	"P2O[14],ULA+,Enabled,Disabled;",
	"D3P2OP,Snow Bug,Disabled,Enabled;",
	"P2-;",
	"P2O[9:8],Video Timings,ULA-48,ULA-128,Pentagon;",
	"P2O[12:10],Memory,Spectrum 128K/+2,Pentagon 1024K,Profi 1024K,Spectrum 48K,Spectrum +2A/+3;",
	"P2-;",
	"P2O[33:32],MMC Mode,Auto(VHD),SD Card 14MHz,SD Card 28MHz;",
	"P2O[31:30],MMC Version,DivMMC+ESXDOS,DivMMC,ZXMMC;",

	"-;",
	"O[37:36],Keyboard,Normal,Ghosting,Recreated ZX,Recr+Ghosting;",
	"O[19:17],Joystick,Kempston,Sinclair I,Sinclair II,Sinclair I+II,Cursor;",
	"O[35:34],Mouse,Disabled,Kempston L/R,Kempston R/L;",
	"O[6],Fast Tape Load,On,Off;",
	"O[1],Tape Sound,On,Off;",
	"O[24:22],CPU Speed,Original,7MHz,14MHz,28MHz,56MHz;",
	"-;",
	"R[0],Reset & Apply;",
	"J,Fire 1,Fire 2;",
	"V,v",`BUILD_DATE
*/

/*const char * menu[11] [32] ={
	{"Audio & Video"},
	{"Hardware"},
	{"Keyboard"},
	{"Joystick"},
	{"Map Keyboard Joystick"},
	{"Mouse"},
	{"Fast Tape Load"},
	{"Tape Sound"},
	{"CPU Speed}",
	{"Reset & Apply"},
	{"END"}
};

const char * avmenu[] ={
	"General Sound",
	"Stereo Mix",
	"PSG/FM",
	"PSG Stereo",
	"PSG Model",	
	"END"
};

const char * hwmenu[] ={
	"Port #FE",
	"Port #FF",
	"ULA+",
	"Snow Bug",
	"Video Timings",	
	"Memory",	
	"MMC Mode",
	"MMC Version",
	"END"
};

const char * gs[] ={
	"512KB",
	"1MB",
	"2MB",
	"Disabled",
	"END"
};
*/

void mapJoystick();

#define MAXMENU 3
#define MAXMENUITEM 12
#define MAXMENUITEMLEN 32

#define MAXLISTIDS 21
#define MAXLISTITEMS 6
#define MAXLISTLEN 20

#define TRUE_VID 2
#define INV_VID 4
#define ZX_BREAK 6
#define ZX_DELETE 8
#define CAPS_LOCK 10
#define ZX_ENTER 12
#define CAPS_SHIFT 14
#define LEFT_ARROW 16
#define RIGHT_ARROW 17
#define UP_ARROW 18
#define DOWN_ARROW 19
#define ZX_EXTEND 20
#define SYM_SHIFT 22
#define ZX_GRAPH 23
#define ZX_SPACE 24

const char keyboard[5] [14] = {
	{ZX_BREAK,ZX_BREAK+1,'1','2','3','4','5','6','7','8','9','0',ZX_DELETE,ZX_DELETE+1},
	{TRUE_VID,TRUE_VID+1,INV_VID,INV_VID+1,'Q','W','E','R','T','Y','U','I','O','P'},
	{CAPS_LOCK,CAPS_LOCK+1,ZX_GRAPH,'A','S','D','F','G','H','J','K','L',ZX_ENTER,ZX_ENTER+1},
	{CAPS_SHIFT,CAPS_SHIFT+1,ZX_EXTEND,ZX_EXTEND+1,'Z','X','C','V','B','N','M',UP_ARROW,CAPS_SHIFT,CAPS_SHIFT+1},
	{SYM_SHIFT,';','\"',',','.',ZX_SPACE,ZX_SPACE+1,ZX_SPACE+2,ZX_SPACE+3,ZX_SPACE+4,LEFT_ARROW,DOWN_ARROW,RIGHT_ARROW,SYM_SHIFT},
};

	

