/*
 * LSM9DS1_registers.h
 *
 *  Created on: Feb 20, 2017
 *      Author: steveb
 */

#ifndef LSM9DS1_REGISTERS_H_
#define LSM9DS1_REGISTERS_H_

typedef uint8_t LSM9DS1_REGISTER;

#define LSM9DS1_REGISTER_ACT_THS          ((LSM9DS1_REGISTER)0x04)
#define LSM9DS1_REGISTER_ACT_DUR          ((LSM9DS1_REGISTER)0x05)
#define LSM9DS1_REGISTER_INT_GEN_CFG_XL   ((LSM9DS1_REGISTER)0x06)
#define LSM9DS1_REGISTER_INT_GEN_THS_X_XL ((LSM9DS1_REGISTER)0x07)
#define LSM9DS1_REGISTER_INT_GEN_THS_Y_XL ((LSM9DS1_REGISTER)0x08)
#define LSM9DS1_REGISTER_INT_GEN_THS_Z_XL ((LSM9DS1_REGISTER)0x09)
#define LSM9DS1_REGISTER_INT_GEN_DUR_XL   ((LSM9DS1_REGISTER)0x0A)
#define LSM9DS1_REGISTER_REFERENCE_G      ((LSM9DS1_REGISTER)0x0B)
#define LSM9DS1_REGISTER_INT1_CTRL        ((LSM9DS1_REGISTER)0x0C)
#define LSM9DS1_REGISTER_INT2_CTRL        ((LSM9DS1_REGISTER)0x0B)
#define LSM9DS1_REGISTER_WHO_AM_I         ((LSM9DS1_REGISTER)0x0F)
#define LSM9DS1_REGISTER_CTRL_REG1_G      ((LSM9DS1_REGISTER)0x10)
#define LSM9DS1_REGISTER_CTRL_REG2_G      ((LSM9DS1_REGISTER)0x11)
#define LSM9DS1_REGISTER_CTRL_REG3_G      ((LSM9DS1_REGISTER)0x12)
#define LSM9DS1_REGISTER_ORIENT_CFG_G     ((LSM9DS1_REGISTER)0x13)
#define LSM9DS1_REGISTER_INT_GEN_SRC_G    ((LSM9DS1_REGISTER)0x14)
#define LSM9DS1_REGISTER_OUT_TEMP_L       ((LSM9DS1_REGISTER)0x15)
#define LSM9DS1_REGISTER_OUT_TEMP_H       ((LSM9DS1_REGISTER)0x16)
#define LSM9DS1_REGISTER_STATUS_REG       ((LSM9DS1_REGISTER)0x17)
#define LSM9DS1_REGISTER_OUT_X_L_G        ((LSM9DS1_REGISTER)0x18)
#define LSM9DS1_REGISTER_OUT_X_H_G        ((LSM9DS1_REGISTER)0x19)
#define LSM9DS1_REGISTER_OUT_Y_L_G        ((LSM9DS1_REGISTER)0x1A)
#define LSM9DS1_REGISTER_OUT_Y_H_G        ((LSM9DS1_REGISTER)0x1B)
#define LSM9DS1_REGISTER_OUT_Z_L_G        ((LSM9DS1_REGISTER)0x1C)
#define LSM9DS1_REGISTER_OUT_Z_H_G        ((LSM9DS1_REGISTER)0x1D)
#define LSM9DS1_REGISTER_CTRL_REG4        ((LSM9DS1_REGISTER)0x1E)
#define LSM9DS1_REGISTER_CTRL_REG5_XL     ((LSM9DS1_REGISTER)0x1F)
#define LSM9DS1_REGISTER_CTRL_REG6_XL     ((LSM9DS1_REGISTER)0x20)
#define LSM9DS1_REGISTER_CTRL_REG7_XL     ((LSM9DS1_REGISTER)0x21)
#define LSM9DS1_REGISTER_CTRL_REG8        ((LSM9DS1_REGISTER)0x22)
#define LSM9DS1_REGISTER_CTRL_REG9        ((LSM9DS1_REGISTER)0x23)
#define LSM9DS1_REGISTER_CTRL_REG10       ((LSM9DS1_REGISTER)0x24)
#define LSM9DS1_REGISTER_INT_GEN_SRC_XL   ((LSM9DS1_REGISTER)0x26)
/*      LSM9DS1_REGISTER_STATUS_REG       ((LSM9DS1_REGISTER)0x27)  /* duplicate of 0x17 */
#define LSM9DS1_REGISTER_OUT_X_L_XL       ((LSM9DS1_REGISTER)0x28)
#define LSM9DS1_REGISTER_OUT_X_H_XL       ((LSM9DS1_REGISTER)0x29)
#define LSM9DS1_REGISTER_OUT_Y_L_XL       ((LSM9DS1_REGISTER)0x2A)
#define LSM9DS1_REGISTER_OUT_Y_H_XL       ((LSM9DS1_REGISTER)0x2B)
#define LSM9DS1_REGISTER_OUT_Z_L_XL       ((LSM9DS1_REGISTER)0x2C)
#define LSM9DS1_REGISTER_OUT_Z_H_XL       ((LSM9DS1_REGISTER)0x2D)
#define LSM9DS1_REGISTER_FIFO_CTRL        ((LSM9DS1_REGISTER)0x2E)
#define LSM9DS1_REGISTER_FIFO_SRC         ((LSM9DS1_REGISTER)0x2F)
#define LSM9DS1_REGISTER_INT_GEN_CFG_G    ((LSM9DS1_REGISTER)0x30)
#define LSM9DS1_REGISTER_INT_GEN_THS_XL_G ((LSM9DS1_REGISTER)0x31)
#define LSM9DS1_REGISTER_INT_GEN_THS_XH_G ((LSM9DS1_REGISTER)0x32)
#define LSM9DS1_REGISTER_INT_GEN_THS_YL_G ((LSM9DS1_REGISTER)0x33)
#define LSM9DS1_REGISTER_INT_GEN_THS_YH_G ((LSM9DS1_REGISTER)0x34)
#define LSM9DS1_REGISTER_INT_GEN_THS_ZL_G ((LSM9DS1_REGISTER)0x35)
#define LSM9DS1_REGISTER_INT_GEN_THS_ZH_G ((LSM9DS1_REGISTER)0x36)
#define LSM9DS1_REGISTER_INT_GEN_DUR_G    ((LSM9DS1_REGISTER)0x37)

#define LSM9DS1_REGISTER_OFFSET_X_REG_L_M ((LSM9DS1_REGISTER)0x05)
#define LSM9DS1_REGISTER_OFFSET_X_REG_H_M ((LSM9DS1_REGISTER)0x06)
#define LSM9DS1_REGISTER_OFFSET_Y_REG_L_M ((LSM9DS1_REGISTER)0x07)
#define LSM9DS1_REGISTER_OFFSET_Y_REG_H_M ((LSM9DS1_REGISTER)0x08)
#define LSM9DS1_REGISTER_OFFSET_Z_REG_L_M ((LSM9DS1_REGISTER)0x09)
#define LSM9DS1_REGISTER_OFFSET_Z_REG_H_M ((LSM9DS1_REGISTER)0x0A)
#define LSM9DS1_REGISTER_WHO_AM_I_M       ((LSM9DS1_REGISTER)0x0F)
#define LSM9DS1_REGISTER_CTRL_REG1_M      ((LSM9DS1_REGISTER)0x20)
#define LSM9DS1_REGISTER_CTRL_REG2_M      ((LSM9DS1_REGISTER)0x21)
#define LSM9DS1_REGISTER_CTRL_REG3_M      ((LSM9DS1_REGISTER)0x22)
#define LSM9DS1_REGISTER_CTRL_REG4_M      ((LSM9DS1_REGISTER)0x23)
#define LSM9DS1_REGISTER_CTRL_REG5_M      ((LSM9DS1_REGISTER)0x24)
#define LSM9DS1_REGISTER_STATUS_REG_M     ((LSM9DS1_REGISTER)0x27)
#define LSM9DS1_REGISTER_OUT_X_L_M        ((LSM9DS1_REGISTER)0x28)
#define LSM9DS1_REGISTER_OUT_X_H_M        ((LSM9DS1_REGISTER)0x29)
#define LSM9DS1_REGISTER_OUT_Y_L_M        ((LSM9DS1_REGISTER)0x2A)
#define LSM9DS1_REGISTER_OUT_Y_H_M        ((LSM9DS1_REGISTER)0x2B)
#define LSM9DS1_REGISTER_OUT_Z_L_M        ((LSM9DS1_REGISTER)0x2C)
#define LSM9DS1_REGISTER_OUT_Z_H_M        ((LSM9DS1_REGISTER)0x2D)
#define LSM9DS1_REGISTER_INT_CFG_M        ((LSM9DS1_REGISTER)0x30)
#define LSM9DS1_REGISTER_INT_SRC_M        ((LSM9DS1_REGISTER)0x31)
#define LSM9DS1_REGISTER_INT_THS_L_M      ((LSM9DS1_REGISTER)0x32)
#define LSM9DS1_REGISTER_INT_THS_H_M      ((LSM9DS1_REGISTER)0x33)

union uint16_to_2uint8
{
  struct uint16_to_2uint8_field
  {
    uint8_t   L;  /* Little Endian byte order means that the least significant byte goes in the lowest address */
    uint8_t   H;
  }         field;
  uint16_t  unsigned_value;
  int16_t   signed_value;
};

union LSM9DS1_transaction
{
  struct LSM9DS1_transaction_field
  {
    struct LSM9DS1_transaction_field_command
    {
      LSM9DS1_REGISTER  AD:6;
      uint8_t           M_S:1;  /* 0=do not increment address, 1=increment address (does not seem to work in general) */
      uint8_t           READ:1; /* 0=write, 1=read */
    }       command;
    union LSM9DS1_transaction_field_body
    {
      struct LSM9DS1_ACT_THS
      {
        uint8_t         ACT_DUR:8;
      }               ACT_THS;
      struct LSM9DS1_ACT_DUR
      {
        uint8_t         ACT_THS:7;
        uint8_t         SLEEP_ON_INACT_EN:1;
      }               ACT_DUR;
      struct LSM9DS1_INT_GEN_CFG_XL
      {
        uint8_t         XLIE_XL:1;
        uint8_t         XHIE_XL:1;
        uint8_t         YLIE_XL:1;
        uint8_t         YHIE_XL:1;
        uint8_t         ZLIE_XL:1;
        uint8_t         ZHIE_XL:1;
        uint8_t         SIXD:1;
        uint8_t         AOI_XL:1;
      }             INT_GEN_CFG_XL;
      struct LSM9DS1_INT_GEN_THS_X_XL
      {
        uint8_t         THS_XL_X:8;
      }               INT_GEN_THS_X_XL;
      struct LSM9DS1_INT_GEN_THS_Y_XL
      {
        uint8_t         THS_XL_Y:8;
      }               INT_GEN_THS_Y_XL;
      struct LSM9DS1_INT_GEN_THS_Z_XL
      {
        uint8_t         THS_XL_Z:8;
      }               INT_GEN_THS_Z_XL;
      struct LSM9DS1_INT_GEN_DUR_XL
      {
        uint8_t         DUR_XL:7;
        uint8_t         WAIT_XL:1;
      }               INT_GEN_DUR_XL;
      struct LSM9DS1_REFERENCE_G
      {
        uint8_t         REF_G:8;
      }               REFERENCE_G;
      struct LSM9DS1_INT1_CTRL
      {
        uint8_t         INT1_DRDY_XL:1;
        uint8_t         INT1_DRDY_G:1;
        uint8_t         INT1_Boot:1;
        uint8_t         INT1_FTH:1;
        uint8_t         INT1_OVR:1;
        uint8_t         INT1_FSS5:1;
        uint8_t         INT1_IG_XL:1;
        uint8_t         INT1_IG_G:1;
      }               INT1_CTRL;
      struct LSM9DS1_INT2_CTRL
      {
        uint8_t         INT2_DRDY_XL:1;
        uint8_t         INT2_DRDY_G:1;
        uint8_t         INT2_DRDY_TEMP:1;
        uint8_t         INT2_FTH:1;
        uint8_t         INT2_OVR:1;
        uint8_t         INT2_FSS5:1;
        uint8_t         zero:1;
        uint8_t         INT2_INACT:1;
      }               INT2_CTRL;
      struct LSM9DS1_WHO_AM_I
      {
        uint8_t         SIX_EIGHT:8;
      }               WHO_AM_I;
      struct LSM9DS1_CTRL_REG1_G
      {
        uint8_t         BW_G:2;
        uint8_t         zero:1;
        uint8_t         FS_G:2;
        uint8_t         ODR_G:3;
      }               CTRL_REG1_G;
      struct LSM9DS1_CTRL_REG2_G
      {
        uint8_t         OUT_SEL:2;
        uint8_t         INT_SEL:2;
        uint8_t         zero:4;
      }               CTRL_REG2_G;
      struct LSM9DS1_CTRL_REG3_G
      {
        uint8_t         HPCFG_G:4;
        uint8_t         zero:2;
        uint8_t         HP_EN:1;
        uint8_t         LP_mode:1;
      }               CTRL_REG3_G;
      struct LSM9DS1_ORIENT_CFG_G
      {
        uint8_t         Orient:3;
        uint8_t         SignZ_G:1;
        uint8_t         SignY_G:1;
        uint8_t         SignX_G:1;
        uint8_t         zero:2;
      }               ORIENT_CFG_G;
      struct LSM9DS1_INT_GEN_SRC_G
      {
        uint8_t         XL_G:1;
        uint8_t         XH_G:1;
        uint8_t         YL_G:1;
        uint8_t         YH_G:1;
        uint8_t         ZL_G:1;
        uint8_t         ZH_G:1;
        uint8_t         IA_G:1;
        uint8_t         zero:1;
      }               INT_GEN_SRC_G;
      struct LSM9DS1_OUT_TEMP_L
      {
        uint8_t         Temp0_7:8;
      }               OUT_TEMP_L;
      struct LSM9DS1_OUT_TEMP_H
      {
        uint8_t         Temp8_11:8;
      }               OUT_TEMP_H;
      struct LSM9DS1_STATUS_REG
      {
        uint8_t         XLDA:1;
        uint8_t         GDA:1;
        uint8_t         TDA:1;
        uint8_t         BOOT_STATUS:1;
        uint8_t         INACT:1;
        uint8_t         IG_G:1;
        uint8_t         IG_XL:1;
        uint8_t         zero:1;
      }               STATUS_REG;
      struct LSM9DS1_OUT_X_L_G
      {
        uint8_t         X_L:8;
      }               OUT_X_L_G;
      struct LSM9DS1_OUT_X_H_G
      {
        uint8_t         X_H:8;
      }               OUT_X_H_G;
      struct LSM9DS1_OUT_Y_L_G
      {
        uint8_t         Y_L:8;
      }               OUT_Y_L_G;
      struct LSM9DS1_OUT_Y_H_G
      {
        uint8_t         Y_H:8;
      }               OUT_Y_H_G;
      struct LSM9DS1_OUT_Z_L_G
      {
        uint8_t         Z_L:8;
      }               OUT_Z_L_G;
      struct LSM9DS1_OUT_Z_H_G
      {
        uint8_t         Z_H:8;
      }               OUT_Z_H_G;
      struct LSM9DS1_CTRL_REG4
      {
        uint8_t         FOURD_XL1:1;
        uint8_t         LIR_XL1:1;
        uint8_t         zero0:1;
        uint8_t         Xen_G:1;
        uint8_t         Yen_G:1;
        uint8_t         Zen_G:1;
        uint8_t         zero1:2;
      }               CTRL_REG4;
      struct LSM9DS1_CTRL_REG5_XL
      {
        uint8_t         zero:3;
        uint8_t         Xen_XL:1;
        uint8_t         Yen_XL:1;
        uint8_t         Zen_XL:1;
        uint8_t         DEC:2;
      }               CTRL_REG5_XL;
      struct LSM9DS1_CTRL_REG6_XL
      {
        uint8_t         BW_XL:2;
        uint8_t         BW_SCAL_ODR:1;
        uint8_t         FS_XL:2;
        uint8_t         ODR_XL:3;
      }               CTRL_REG6_XL;
      struct LSM9DS1_CTRL_REG7_XL
      {
        uint8_t         HPIS1:1;
        uint8_t         zero0:1;
        uint8_t         FDS:1;
        uint8_t         zero1:2;
        uint8_t         DCF:2;
        uint8_t         HR:1;
      }               CTRL_REG7_XL;
      struct LSM9DS1_CTRL_REG8
      {
        uint8_t         SW_RESET:1;
        uint8_t         BLE:1;
        uint8_t         IF_ADD_INC:1;
        uint8_t         SIM:1;
        uint8_t         PP_OD:1;
        uint8_t         H_LACTIVE:1;
        uint8_t         BDU:1;
        uint8_t         BOOT:1;
      }               CTRL_REG8;
      struct LSM9DS1_CTRL_REG9
      {
        uint8_t         STOP_ON_FTH:1;
        uint8_t         FIFO_EN:1;
        uint8_t         I2C_DISABLE:1;
        uint8_t         DRDY_mask_bit:1;
        uint8_t         FIFO_TEMP_EN:1;
        uint8_t         zero0:1;
        uint8_t         SLEEP_G:1;
        uint8_t         zero1:1;
      }               CTRL_REG9;
      struct LSM9DS1_CTRL_REG10
      {
        uint8_t         ST_XL:1;
        uint8_t         zero0:1;
        uint8_t         ST_G:1;
        uint8_t         zero1:5;
      }               CTRL_REG10;
      struct LSM9DS1_INT_GEN_SRC_XL
      {
        uint8_t         XL_XL:1;
        uint8_t         XH_XL:1;
        uint8_t         YL_XL:1;
        uint8_t         YH_XL:1;
        uint8_t         ZL_XL:1;
        uint8_t         ZH_XL:1;
        uint8_t         IA_XL:1;
        uint8_t         zero:1;
      }               INT_GEN_SRC_XL;
      struct LSM9DS1_OUT_X_L_XL
      {
        uint8_t         X_L_XL:8;
      }               OUT_X_L_XL;
      struct LSM9DS1_OUT_X_H_XL
      {
        uint8_t         X_H_XL:8;
      }               OUT_X_H_XL;
      struct LSM9DS1_OUT_Y_L_XL
      {
        uint8_t         Y_L_XL:8;
      }               OUT_Y_L_XL;
      struct LSM9DS1_OUT_Y_H_XL
      {
        uint8_t         Y_H_XL:8;
      }               OUT_Y_H_XL;
      struct LSM9DS1_OUT_Z_L_XL
      {
        uint8_t         Z_L_XL:8;
      }               OUT_Z_L_XL;
      struct LSM9DS1_OUT_Z_H_XL
      {
        uint8_t         Z_H_XL:8;
      }               OUT_Z_H_XL;
      struct LSM9DS1_FIFO_CTRL
      {
        uint8_t         FTH:5;
        uint8_t         FMODE:3;
      }               FIFO_CTRL;
      struct LSM9DS1_FIFO_SRC
      {
        uint8_t         FSS:6;
        uint8_t         OVRN:1;
        uint8_t         FTH:1;
      }               FIFO_SRC;
      struct LSM9DS1_INT_GEN_CFG_G
      {
        uint8_t         XLIE_G:1;
        uint8_t         XHIE_G:1;
        uint8_t         YLIE_G:1;
        uint8_t         YHIE_G:1;
        uint8_t         ZLIE_G:1;
        uint8_t         ZhIE_G:1;
        uint8_t         LIR_G:1;
        uint8_t         AOI_G:1;
      }               INT_GEN_CFG_G;
      struct LSM9DS1_INT_GEN_THS_XL_G
      {
        uint8_t         THS_G_X0_7:8;
      }               INT_GEN_THS_XL_G;
      struct LSM9DS1_INT_GEN_THS_XH_G
      {
        uint8_t        THS_G_X8_14:7;
        uint8_t        DCRM_G:1;
      }               INT_GEN_THS_XH_G;
      struct LSM9DS1_INT_GEN_THS_YL_G
      {
        uint8_t         THS_G_Y0_7:8;
      }               INT_GEN_THS_YL_G;
      struct LSM9DS1_INT_GEN_THS_YH_G
      {
        uint8_t         THS_G_Y8_14:7;
        uint8_t         zero:1;
      }               INT_GEN_THS_YH_G;
      struct LSM9DS1_INT_GEN_THS_ZL_G
      {
        uint8_t         THS_G_Z0_7:8;
      }               INT_GEN_THS_ZL_G;
      struct LSM9DS1_INT_GEN_THS_ZH_G
      {
        uint8_t         THS_G_Z8_14:7;
        uint8_t         zero1:1;
      }               INT_GEN_THS_ZH_G;
      struct LSM9DS1_INT_GEN_DUR_G
      {
        uint8_t         DUR_G:7;
        uint8_t         WAIT_G:1;
      }               INT_GEN_DUR_G;
      struct LSM9DS1_OFFSET_X_REG_L_M
      {
        uint8_t         OFXM0_7:8;
      }               OFFSET_X_REG_L_M;
      struct LSM9DS1_OFFSET_X_REG_H_M
      {
        uint8_t         OFXM8_15:8;
      }               OFFSET_X_REG_H_M;
      struct LSM9DS1_OFFSET_Y_REG_L_M
      {
        uint8_t         OFYM0_7:8;
      }               OFFSET_Y_REG_L_M;
      struct LSM9DS1_OFFSET_Y_REG_H_M
      {
        uint8_t         OFYM8_15:8;
      }               OFFSET_Y_REG_H_M;
      struct LSM9DS1_OFFSET_Z_REG_L_M
      {
        uint8_t         OFZM0_7:8;
      }               OFFSET_Z_REG_L_M;
      struct LSM9DS1_OFFSET_Z_REG_H_M
      {
        uint8_t         OFZM8_15:8;
      }               OFFSET_Z_REG_H_M;
      struct LSM9DS1_WHO_AM_I_M
      {
        uint8_t         THREE_D:8;
      }               WHO_AM_I_M;
      struct LSM9DS1_CTRL_REG1_M
      {
        uint8_t         ST:1;
        uint8_t         zero:1;
        uint8_t         DO:3;
        uint8_t         OM:2;
        uint8_t         TEMP_COMP:1;
      }               CTRL_REG1_M;
      struct LSM9DS1_CTRL_REG2_M
      {
        uint8_t         zero0:2;
        uint8_t         SOFT_RST:1;
        uint8_t         REBOOT:1;
        uint8_t         zero1:1;
        uint8_t         FS:2;
        uint8_t         zero2:1;
      }               CTRL_REG2_M;
      struct LSM9DS1_CTRL_REG3_M
      {
        uint8_t         MD:2;
        uint8_t         SIM:1;
        uint8_t         zero0:2;
        uint8_t         LP:1;
        uint8_t         zero1:1;
        uint8_t         I2C_DISABLE:1;
      }               CTRL_REG3_M;
      struct LSM9DS1_CTRL_REG4_M
      {
        uint8_t         zero0:1;
        uint8_t         BLE:1;
        uint8_t         OMZ:2;
        uint8_t         zero1:4;
      }               CTRL_REG4_M;
      struct LSM9DS1_CTRL_REG5_M
      {
        uint8_t         zero0:6;
        uint8_t         BDU:1;
        uint8_t         zero1:1;
      }               CTRL_REG5_M;
      struct LSM9DS1_STATUS_REG_M
      {
        uint8_t         XDA:1;
        uint8_t         YDA:1;
        uint8_t         ZDA:1;
        uint8_t         ZYXDA:1;
        uint8_t         XOR:1;
        uint8_t         YOR:1;
        uint8_t         ZOR:1;
        uint8_t         ZYXOR:1;
      }               STATUS_REG_M;
      struct LSM9DS1_OUT_X_L_M
      {
        uint8_t         X_L_M:8;
      }               OUT_X_L_M;
      struct LSM9DS1_OUT_X_H_M
      {
        uint8_t         X_H_M:8;
      }               OUT_X_H_M;
      struct LSM9DS1_OUT_Y_L_M
      {
        uint8_t         Y_L_M:8;
      }               OUT_Y_L_M;
      struct LSM9DS1_OUT_Y_H_M
      {
        uint8_t         Y_H_M:8;
      }               OUT_Y_H_M;
      struct LSM9DS1_OUT_Z_L_M
      {
        uint8_t         Z_L_M:8;
      }               OUT_Z_L_M;
      struct LSM9DS1_OUT_Z_H_M
      {
        uint8_t         Z_H_M:8;
      }               OUT_Z_H_M;
      struct LSM9DS1_INT_CFG_M
      {
        uint8_t         IEN:1;
        uint8_t         IEL:1;
        uint8_t         IEA:1;
        uint8_t         zero:2;
        uint8_t         ZIEN:1;
        uint8_t         YIEN:1;
        uint8_t         XIEN:1;
      }               INT_CFG_M;
      struct LSM9DS1_INT_SRC_M
      {
        uint8_t         INT:1;
        uint8_t         MROI:1;
        uint8_t         NTH_Z:1;
        uint8_t         NTH_Y:1;
        uint8_t         NTH_X:1;
        uint8_t         PTH_Z:1;
        uint8_t         PTH_Y:1;
        uint8_t         PTH_X:1;
      }               INT_SRC_M;
      struct LSM9DS1_INT_THS_L_M
      {
        uint8_t         THS8_7:8;
      }               INT_THS_L_M;
      struct LSM9DS1_INT_THS_H_M
      {
        uint8_t         THS8_14:7;
        uint8_t         zero:1;
      }               INT_THS_H_M;
    }       body;
  }       field;
  uint8_t value[1+1];
};

#endif /* LSM9DS1_REGISTERS_H_ */
