// See also MPU-9255 Register Map and Descriptions, Revision 4.0,
// RM-MPU-9255A-00, Rev. 1.4, 9/9/2013 for registers not listed in above
// document; the MPU9255 and MPU9150 are virtually identical but the latter has
// a different register map

// Using the MPU-9255 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define MPU9255_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU9255_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS  0x0C   // Address of magnetometer
#endif // AD0

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // (AKA WIA) should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define XG_OFFSET_H       0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L       0x14
#define YG_OFFSET_H       0x15
#define YG_OFFSET_L       0x16
#define ZG_OFFSET_H       0x17
#define ZG_OFFSET_L       0x18
#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define LP_ACCEL_ODR      0x1E
#define WOM_THR           0x1F

// Duration counter threshold for motion interrupt generation, 1 kHz rate,
// LSB = 1 ms
#define MOT_DUR           0x20
// Zero-motion detection threshold bits [7:0]
#define ZMOT_THR          0x21
// Duration counter threshold for zero motion interrupt generation, 16 Hz rate,
// LSB = 64 ms
#define ZRMOT_DUR         0x22

#define FIFO_EN            0x23
#define I2C_MST_CTRL       0x24
#define I2C_SLV0_ADDR      0x25
#define I2C_SLV0_REG       0x26
#define I2C_SLV0_CTRL      0x27
#define I2C_SLV1_ADDR      0x28
#define I2C_SLV1_REG       0x29
#define I2C_SLV1_CTRL      0x2A
#define I2C_SLV2_ADDR      0x2B
#define I2C_SLV2_REG       0x2C
#define I2C_SLV2_CTRL      0x2D
#define I2C_SLV3_ADDR      0x2E
#define I2C_SLV3_REG       0x2F
#define I2C_SLV3_CTRL      0x30
#define I2C_SLV4_ADDR      0x31
#define I2C_SLV4_REG       0x32
#define I2C_SLV4_DO        0x33
#define I2C_SLV4_CTRL      0x34
#define I2C_SLV4_DI        0x35
#define I2C_MST_STATUS     0x36
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define DMP_INT_STATUS     0x39  // Check DMP interrupt
#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define TEMP_OUT_H         0x41
#define TEMP_OUT_L         0x42
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48
#define EXT_SENS_DATA_00   0x49
#define EXT_SENS_DATA_01   0x4A
#define EXT_SENS_DATA_02   0x4B
#define EXT_SENS_DATA_03   0x4C
#define EXT_SENS_DATA_04   0x4D
#define EXT_SENS_DATA_05   0x4E
#define EXT_SENS_DATA_06   0x4F
#define EXT_SENS_DATA_07   0x50
#define EXT_SENS_DATA_08   0x51
#define EXT_SENS_DATA_09   0x52
#define EXT_SENS_DATA_10   0x53
#define EXT_SENS_DATA_11   0x54
#define EXT_SENS_DATA_12   0x55
#define EXT_SENS_DATA_13   0x56
#define EXT_SENS_DATA_14   0x57
#define EXT_SENS_DATA_15   0x58
#define EXT_SENS_DATA_16   0x59
#define EXT_SENS_DATA_17   0x5A
#define EXT_SENS_DATA_18   0x5B
#define EXT_SENS_DATA_19   0x5C
#define EXT_SENS_DATA_20   0x5D
#define EXT_SENS_DATA_21   0x5E
#define EXT_SENS_DATA_22   0x5F
#define EXT_SENS_DATA_23   0x60
#define MOT_DETECT_STATUS  0x61
#define I2C_SLV0_DO        0x63
#define I2C_SLV1_DO        0x64
#define I2C_SLV2_DO        0x65
#define I2C_SLV3_DO        0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL    0x69
#define USER_CTRL          0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1         0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x6C
#define DMP_BANK           0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT         0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG            0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1          0x70
#define DMP_REG_2          0x71
#define FIFO_COUNTH        0x72
#define FIFO_COUNTL        0x73
#define FIFO_R_W           0x74
#define WHO_AM_I_MPU9255   0x75 // Should return 0x75
#define XA_OFFSET_H        0x77
#define XA_OFFSET_L        0x78
#define YA_OFFSET_H        0x7A
#define YA_OFFSET_L        0x7B
#define ZA_OFFSET_H        0x7D
#define ZA_OFFSET_L        0x7E

#define PWR1_DEVICE_RESET_BIT   7
#define PWR1_SLEEP_BIT          6
#define PWR1_CYCLE_BIT          5
#define PWR1_GYRO_STBY			4
#define PWR1_PD_PTAT			3
#define PWR1_CLKSEL_BIT         2
#define PWR1_CLKSEL_LENGTH      3

#define CLOCK_INTERNAL          0x00
#define AUTO_CLK_OR_PLL         0x01
#define AUTO_CLK_OR_PLL         0x02
#define AUTO_CLK_OR_PLL         0x03
#define AUTO_CLK_OR_PLL         0x04
#define AUTO_CLK_OR_PLL         0x05
#define CLOCK_INTERNAL          0x06
#define CLOCK_KEEP_RESET        0x07

#define GCONFIG_XG_ST_BIT		7
#define GCONFIG_YG_ST_BIT		6
#define GCONFIG_ZG_ST_BIT		5
#define GCONFIG_FS_SEL_BIT      4
#define GCONFIG_FS_SEL_LENGTH   2

#define GYRO_FS_250         0x00
#define GYRO_FS_500         0x01
#define GYRO_FS_1000        0x02
#define GYRO_FS_2000        0x03

#define ACONFIG_XA_ST_BIT           7
#define ACONFIG_YA_ST_BIT           6
#define ACONFIG_ZA_ST_BIT           5
#define ACONFIG_AFS_SEL_BIT         4
#define ACONFIG_AFS_SEL_LENGTH      2

#define ACCEL_FS_2          0x00
#define ACCEL_FS_4          0x01
#define ACCEL_FS_8          0x02
#define ACCEL_FS_16     	0x03

#define WHO_AM_I_BIT        7
#define WHO_AM_I_LENGTH     8

#define CFG_FIFO_MODE_BIT		6
#define CFG_EXT_SYNC_SET_BIT    5
#define CFG_EXT_SYNC_SET_LENGTH 3
#define CFG_DLPF_CFG_BIT		2
#define CFG_DLPF_CFG_LENGTH		3

#define ACONFIG2_DLPF_CFG_BIT	 2
#define ACONFIG2_DLPF_CFG_LENGTH 3

#define ACCEL_DLPF_BW_460		0x00
#define ACCEL_DLPF_BW_184		0x01
#define ACCEL_DLPF_BW_92		0x02
#define ACCEL_DLPF_BW_41		0x03
#define ACCEL_DLPF_BW_20		0x04
#define ACCEL_DLPF_BW_10		0x05
#define ACCEL_DLPF_BW_5			0x06

#define DLPF_BW_250         0x00
#define DLPF_BW_184         0x01
#define DLPF_BW_92          0x02
#define DLPF_BW_41          0x03
#define DLPF_BW_20          0x04
#define DLPF_BW_10          0x05
#define DLPF_BW_5           0x06

#define TEMP_FIFO_EN_BIT    7
#define XG_FIFO_EN_BIT      6
#define YG_FIFO_EN_BIT      5
#define ZG_FIFO_EN_BIT      4
#define ACCEL_FIFO_EN_BIT   3
#define SLV2_FIFO_EN_BIT    2
#define SLV1_FIFO_EN_BIT    1
#define SLV0_FIFO_EN_BIT    0

#define MULT_MST_EN_BIT     7
#define WAIT_FOR_ES_BIT     6
#define SLV_3_FIFO_EN_BIT   5
#define I2C_MST_P_NSR_BIT   4
#define I2C_MST_CLK_BIT     3
#define I2C_MST_CLK_LENGTH  4

#define I2C_SLV_RW_BIT      7
#define I2C_SLV_ADDR_BIT    6
#define I2C_SLV_ADDR_LENGTH 7
#define I2C_SLV_EN_BIT      7
#define I2C_SLV_BYTE_SW_BIT 6
#define I2C_SLV_REG_DIS_BIT 5
#define I2C_SLV_GRP_BIT     4
#define I2C_SLV_LEN_BIT     3
#define I2C_SLV_LEN_LENGTH 	4

#define I2C_SLV4_RW_BIT         7
#define I2C_SLV4_ADDR_BIT       6
#define I2C_SLV4_ADDR_LENGTH    7
#define I2C_SLV4_EN_BIT         7
#define I2C_SLV4_INT_EN_BIT     6
#define I2C_SLV4_REG_DIS_BIT    5
#define I2C_SLV4_MST_DLY_BIT    4
#define I2C_SLV4_MST_DLY_LENGTH 5

#define MST_PASS_THROUGH_BIT    7
#define MST_I2C_SLV4_DONE_BIT   6
#define MST_I2C_LOST_ARB_BIT    5
#define MST_I2C_SLV4_NACK_BIT   4
#define MST_I2C_SLV3_NACK_BIT   3
#define MST_I2C_SLV2_NACK_BIT   2
#define MST_I2C_SLV1_NACK_BIT   1
#define MST_I2C_SLV0_NACK_BIT   0

#define INTCFG_INT_LEVEL_BIT        7
#define INTCFG_INT_OPEN_BIT         6
#define INTCFG_LATCH_INT_EN_BIT     5
#define INTCFG_INT_RD_CLEAR_BIT     4
#define INTCFG_FSYNC_INT_LEVEL_BIT  3
#define INTCFG_FSYNC_INT_EN_BIT     2
#define INTCFG_I2C_BYPASS_EN_BIT    1
#define INTCFG_CLKOUT_EN_BIT        0


#define INTERRUPT_WOM_BIT           6
#define INTERRUPT_FIFO_OFLOW_BIT    4
#define INTERRUPT_FSYNC_INT_BIT     3
#define INTERRUPT_DATA_RDY_BIT      0

#define MOTION_MOT_XNEG_BIT     7
#define MOTION_MOT_XPOS_BIT     6
#define MOTION_MOT_YNEG_BIT     5
#define MOTION_MOT_YPOS_BIT     4
#define MOTION_MOT_ZNEG_BIT     3
#define MOTION_MOT_ZPOS_BIT     2
#define MOTION_MOT_ZRMOT_BIT    0

#define DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define PATHRESET_GYRO_RESET_BIT    2
#define PATHRESET_ACCEL_RESET_BIT   1
#define PATHRESET_TEMP_RESET_BIT    0

#define DETECT_ACCEL_ON_DELAY_BIT       5
#define DETECT_ACCEL_ON_DELAY_LENGTH    2
#define DETECT_FF_COUNT_BIT             3
#define DETECT_FF_COUNT_LENGTH          2
#define DETECT_MOT_COUNT_BIT            1
#define DETECT_MOT_COUNT_LENGTH         2

#define DETECT_DECREMENT_RESET  0x0
#define DETECT_DECREMENT_1      0x1
#define DETECT_DECREMENT_2      0x2
#define DETECT_DECREMENT_4      0x3

#define USERCTRL_DMP_EN_BIT    			7
#define USERCTRL_FIFO_EN_BIT            6
#define USERCTRL_I2C_MST_EN_BIT         5
#define USERCTRL_I2C_IF_DIS_BIT         4
#define USERCTRL_DMP_RESET_BIT          3
#define USERCTRL_FIFO_RESET_BIT         2
#define USERCTRL_I2C_MST_RESET_BIT      1
#define USERCTRL_SIG_COND_RESET_BIT     0



#define PWR2_STBY_XA_BIT            5
#define PWR2_STBY_YA_BIT            4
#define PWR2_STBY_ZA_BIT            3
#define PWR2_STBY_XG_BIT            2
#define PWR2_STBY_YG_BIT            1
#define PWR2_STBY_ZG_BIT            0

#define DMPINT_5_BIT            5
#define DMPINT_4_BIT            4
#define DMPINT_3_BIT            3
#define DMPINT_2_BIT            2
#define DMPINT_1_BIT            1
#define DMPINT_0_BIT            0

#define DMP_MEMORY_BANKS        8
#define DMP_MEMORY_BANK_SIZE    256
#define DMP_MEMORY_CHUNK_SIZE   16
