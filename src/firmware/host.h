#ifndef HOST_H
#define HOST_H

typedef unsigned short uint16_t;
typedef unsigned long uint32_t;
//typedef unsigned long long uint64_t;
typedef unsigned char uint8_t;

//#define XDEBUG 1

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define SLOT_SNA 		0


#define OSD_RAM_8 0x20000000		//start address of 2048x8 byte OSD RAM
#define OSD_RW_8(x) *(volatile unsigned char *)(OSD_RAM_8+x)


#define IO_PORTS 0x30000000		
#define IO_RW(x) *(volatile unsigned int *)(IO_PORTS+x)


#define IO_RAM_STATE	0xD0

#define IO_KB_ON 	0xE0
#define IO_MENU_ON 	0xE4
#define PAUSE_Z80	0xE8	//Pause the Z80 1=pause 0=run
#define IO_DEBUG_ON	0xEC	//Pause the Z80 1=pause 0=run

#define IO_JOYPAD 	0xF0
#define IO_IOCTL_STATE 	0x800
#define IO_IOCTL_ADDR 	0x804
#define IO_ACK_FLAG 	0x808
#define IO_TAP_DATA 	0x80C
#define IO_TAP_ADDR 	0x810

#define IO_KEYB_0 	0x100
#define IO_KEYB_1 	0x104
#define IO_KEYB_2 	0x108
#define IO_KEYB_3 	0x10c
#define IO_KEYB_4 	0x110
#define IO_KEYB_5 	0x114
#define IO_KEYB_6 	0x118
#define IO_KEYB_7 	0x11c

#define IO_STATUS0 	0x200
#define IO_STATUS1 	0x204
#define IO_STATUSC  0x208  //status change flag (if 1 when read then core has requested a status change - write 1 to acknowledge)

#define IO_REQ_ADDR 0x300

#define IO_JOYK_0_0 	0x400
#define IO_JOYK_0_1 	0x404
#define IO_JOYK_0_2 	0x408
#define IO_JOYK_0_3 	0x40C
#define IO_JOYK_0_4 	0x410
#define IO_JOYK_0_5 	0x414
#define IO_JOYK_0_6 	0x418
#define IO_JOYK_0_7 	0x41C
#define IO_JOYK_1_0 	0x420
#define IO_JOYK_1_1 	0x424
#define IO_JOYK_1_2 	0x428
#define IO_JOYK_1_3 	0x42C
#define IO_JOYK_1_4 	0x430
#define IO_JOYK_1_5 	0x434
#define IO_JOYK_1_6 	0x438
#define IO_JOYK_1_7 	0x43C
#define IO_JOYK_2_0 	0x440
#define IO_JOYK_2_1 	0x444
#define IO_JOYK_2_2 	0x448
#define IO_JOYK_2_3 	0x44C
#define IO_JOYK_2_4 	0x450
#define IO_JOYK_2_5 	0x454
#define IO_JOYK_2_6 	0x458
#define IO_JOYK_2_7 	0x45C
#define IO_JOYK_3_0 	0x460
#define IO_JOYK_3_1 	0x464
#define IO_JOYK_3_2 	0x468
#define IO_JOYK_3_3 	0x46C
#define IO_JOYK_3_4 	0x470
#define IO_JOYK_3_5 	0x474
#define IO_JOYK_3_6 	0x478
#define IO_JOYK_3_7 	0x47C
#define IO_JOYK_4_0 	0x480
#define IO_JOYK_4_1 	0x484
#define IO_JOYK_4_2 	0x488
#define IO_JOYK_4_3 	0x48C
#define IO_JOYK_4_4 	0x490
#define IO_JOYK_4_5 	0x494
#define IO_JOYK_4_6 	0x498
#define IO_JOYK_4_7 	0x49C
#define IO_JOYK_5_0 	0x4A0
#define IO_JOYK_5_1 	0x4A4
#define IO_JOYK_5_2 	0x4A8
#define IO_JOYK_5_3 	0x4AC
#define IO_JOYK_5_4 	0x4B0
#define IO_JOYK_5_5 	0x4B4
#define IO_JOYK_5_6 	0x4B8
#define IO_JOYK_5_7 	0x4BC
#define IO_JOYK_6_0 	0x4C0
#define IO_JOYK_6_1 	0x4C4
#define IO_JOYK_6_2 	0x4C8
#define IO_JOYK_6_3 	0x4CC
#define IO_JOYK_6_4 	0x4D0
#define IO_JOYK_6_5 	0x4D4
#define IO_JOYK_6_6 	0x4D8
#define IO_JOYK_6_7 	0x4DC
#define IO_JOYK_7_0 	0x4E0
#define IO_JOYK_7_1 	0x4E4
#define IO_JOYK_7_2 	0x4E8
#define IO_JOYK_7_3 	0x4EC
#define IO_JOYK_7_4 	0x4F0
#define IO_JOYK_7_5 	0x4F4
#define IO_JOYK_7_6 	0x4F8
#define IO_JOYK_7_7 	0x4FC
#define IO_JOYK_8_0 	0x500
#define IO_JOYK_8_1 	0x504
#define IO_JOYK_8_2 	0x508
#define IO_JOYK_8_3 	0x50C
#define IO_JOYK_8_4 	0x510
#define IO_JOYK_8_5 	0x514
#define IO_JOYK_8_6 	0x518
#define IO_JOYK_8_7 	0x51C
#define IO_JOYK_9_0 	0x520
#define IO_JOYK_9_1 	0x524
#define IO_JOYK_9_2 	0x528
#define IO_JOYK_9_3 	0x52C
#define IO_JOYK_9_4 	0x530
#define IO_JOYK_9_5 	0x534
#define IO_JOYK_9_6 	0x538
#define IO_JOYK_9_7 	0x53C

#define SEND_FKEYS	 	0x600




#define DPAD_UP		0x01
#define DPAD_DOWN	0x02
#define DPAD_LEFT	0x04
#define DPAD_RIGHT	0x08
#define FACE_A		0x10
#define FACE_B		0x20
#define FACE_X		0x40
#define FACE_Y		0x80
#define L_TRIG		0x100
#define R_TRIG		0x200
#define SELECT		0x4000
#define START		0x8000


#define IOCTL 0x40000000
#define IOCTL_RW(x) *(volatile unsigned int *)(IOCTL+x)
#define SET_SIZE		0x00		//size to read/write
#define SET_ADDR_U		0x04		//high 16 bits of offset address
#define SET_ADDR_L		0x08		//low bits of offset address
#define SET_ID			0x0c
#define DOWNLOAD		0x10
#define UPLOAD			0x14
#define ROM_LOADED		0x18
#define REQ_ACK			0x1c
#define COMPLETE_ACK	0x20
#define SET_INDEX		0x24
#define DISK_BUFF_ADDR	0x100
#define DISK_BUFF_RD	0x104
#define DISK_BUFF_WR	0x108
#define DISK_MOUNTED	0x10c  //bit 0 = mounted = bit 1 (0=RO 1=RW)
#define DISK_SIZE_LOW	0x110
#define DISK_SIZE_HIGH	0x114
#define VHD_MOUNTED		0x118  //bit 0 = mounted = bit 1 (0=RO 1=RW)
#define VHD_BUFF_ADDR	0x11c
#define FNAME_ADDR		0x200
#define FNAME_DATA		0x204
#define FNAME_WRITE		0x208



#define BRIDGE 0x50000000		
#define BRIDGE_RW(x) *(volatile unsigned int *)(BRIDGE+x)
#define TARGET_0 0x00
#define TARGET_4 0x04
#define TARGET_8 0x08
#define TARGET_20 0x20
#define TARGET_24 0x24
#define TARGET_28 0x28
#define TARGET_2C 0x2c
#define TARGET_40 0x40
#define TARGET_44 0x44
#define TARGET_48 0x48
#define TARGET_4C 0x4c

#define UPDATED_SLOTS	0x80

#define IO_HOST 0x60000000		
#define HOST_RW(x) *(volatile unsigned char *)(IO_HOST+x)

#define IO_CPUMEM 0x90000000	
#define CPUMEM_RW(x) *(volatile unsigned char *)(IO_CPUMEM+x)



#endif

