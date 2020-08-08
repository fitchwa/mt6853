#ifndef H532B_LCD
#define H532B_LCD

#if 1
#define DBG_PRINTK(fmt, args...) printk("H532B LCD %s: " fmt "\n", __func__, ##args)
#else
#define DBG_PRINTK(ARGS...)		do {} while(0);
#endif

#define DEVICE_NAME			"h532blcdc"
#define CLASS_NAME			"h532blcdc"

#define Direction_Normal	0x1
#define	Direction_Inverse	0x2

#define LCD_COMMAND           0
#define LCD_DATA              1

// LCD Instruction Definition
#define LCD_EXT_SET_LOW					0x30
#define LCE_EXT_SET_HIGH				0x31

// if EXT is LOW
#define LCD_DIS_ON		0xAF	// Display On
#define LCD_DIS_OFF		0xAE	// Display Off
#define LCD_DIS_NOR		0xA6	// Normal Display
#define LCD_DIS_INV		0xA7	// Inverse Display
#define LCD_COM_SCN		0xBB	// COM Scan Direction
#define LCD_DIS_CTRL	0xCA	// Display Control
#define LCD_SLP_IN		0x95	// Sleep In
#define LCD_SLP_OUT		0x94	// Sleep Out
#define LCD_LA_SET		0x75	// Line Address Set
#define LCD_CA_SET		0x15	// Column Address Set
#define LCD_DAT_SDR		0xBC	// Data Scan Direction
#define LCD_RAM_WR		0x5C	// Memory Write
#define LCD_RAM_RD		0x5D	// Memory Read
#define LCD_PTL_IN		0xA8	// Partial In
#define LCD_PTL_OUT		0xA9	// Partial Out
#define LCD_RMW_IN		0xE0	// Read Modify Write In
#define LCD_RMW_END		0xEE	// Read Modify Write Out
#define LCD_ASC_SET		0xAA	// Area Scroll Set
#define LCD_SC_START	0xAB	// Scroll Start Address Set
#define LCD_OSC_ON		0xD1	// Internal Oscillation On
#define LCD_OSC_OFF		0xD2	// Internal Oscillation Off
#define LCD_PWR_CTRL	0x20	// Power Control Set
#define LCD_VOL_CTRL	0x81	// Electronic Volume Control
#define LCD_VOL_UP		0xD6	// Increment Electronic Control
#define LCD_VOL_DOWN	0xD7	// Decrement Electronic Control

#define LCD_NOP			0x25	// NOP( Non Operating )
#define LCD_EP_INT		0x07	// Initial Code

// if EXT is HIGH
#define LCD_GRAY1_SET	0x20	// Set Gray 1 Value
#define LCD_GRAY2_SET	0x21	// Set Gray 2 Value
#define LCD_ANA_SET		0x32	// Analog Circuit Set
#define LCD_SW_INT		0x34	// Software Initial


// IO Control Parameter Definition
#define LCD_IOCTL_DISPLAY_ON			0x01
#define LCD_IOCTL_DISPLAY_OFF			0x02
#define LCD_IOCTL_BACKLIGHT_ON			0x0B
#define LCD_IOCTL_BACKLIGHT_OFF			0x0C
#define LCD_IOCTL_DISPLAY_DATA			0x10
#define LCD_IOCTL_SET_DIRECTION			0x11

#define Direction_Normal	0x1
#define	Direction_Inverse	0x2

// Display Control Definition
#define MAX_CHAR			38		// 鞝勳泊 LCD 鞐?於滊牓 臧€電ロ暅 斓滊寑 氍胳瀽靾?
#define MAX_LINE			4		// 鞝勳泊 LCD 鞐?於滊牓 臧€電ロ暅 斓滊寑 霛检澑 靾?
#define MAX_COLUMN			8		// 頃?霛检澑鞐?於滊牓 臧€電ロ暅 斓滊寑 氍胳瀽靾?
#define FONT_HEIGHT			16		// 韽绊姼 雴掛澊
#define FONT_WIDTH			16		// 韽绊姼 雱堧箘


// Resolution 200 * 40
#define	LCD_COLUMN_END		85		// 85*3=255
#define LCD_LINE_END		40		//

unsigned short usMask[3] = {0x001F, 0x07C0, 0xF800};	// Normal Output
unsigned short usMask_Inverse[3] = {0xF800, 0x07C0, 0x001F};	// Inverse Oputput

/* LCD ioctl commands */
typedef struct
{
	unsigned char data[32*40]; // 1280
} __attribute__((packed))pixData;

#define H532BLCD_SEND_DATA				_IOW('C', 0x01, pixData)
#define H532BLCD_SET_POWER				_IOW('C', 0x02, int *)
#define H532BLCD_SET_INV_SCREEN			_IOW('C', 0x03, int *)
#define H532BLCD_SET_BACKLIGHT			_IOW('C', 0x04, int *)
#define H532BLCD_GET_POWER				_IOR('C', 0x05, int *)
#define H532BLCD_GET_INV_SCREEN			_IOR('C', 0x06, int *)
#define H532BLCD_GET_BACKLIGHT			_IOR('C', 0x07, int *)

/* LCD device driver data */
struct h532b_lcd_pin {
	unsigned int a0;
	unsigned int backlight_gpio;
	unsigned int reset_gpio;
	unsigned int power_ldo;
	unsigned int power_level_shift_ldo;

	wait_queue_head_t wq;
	int wi_flag;
};

#endif
