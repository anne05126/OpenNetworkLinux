#ifndef __X86_64_ALPHANETWORKS_SPX70D0_082F_GPIO_TABLE_H__
#define __X86_64_ALPHANETWORKS_SPX70D0_082F_GPIO_TABLE_H__

#define SPX70D0_082F_PCA953x_GPIO(P1, P2)            (P1*8+P2)

/* XFP 1~4 port */
#define SPX70D0_082F_I2C_PCA9539_1_BASE                   432

/* XFP 5~8 port */
#define SPX70D0_082F_I2C_PCA9539_4_BASE                   448

/* QSFP 9~10 port */
#define SPX70D0_082F_I2C_PCA9539_3_BASE                   464


#define SPX70D0_082F_PCA9539_GPIO_XFP_1_TX_DIS_N    (SPX70D0_082F_I2C_PCA9539_1_BASE + SPX70D0_082F_PCA953x_GPIO(0,0))
#define SPX70D0_082F_PCA9539_GPIO_XFP_1_TX_FAULT_N  (SPX70D0_082F_I2C_PCA9539_1_BASE + SPX70D0_082F_PCA953x_GPIO(0,1))
#define SPX70D0_082F_PCA9539_GPIO_XFP_1_PRSNT_N     (SPX70D0_082F_I2C_PCA9539_1_BASE + SPX70D0_082F_PCA953x_GPIO(0,2))
#define SPX70D0_082F_PCA9539_GPIO_XFP_2_TX_DIS_N    (SPX70D0_082F_I2C_PCA9539_1_BASE + SPX70D0_082F_PCA953x_GPIO(0,4))
#define SPX70D0_082F_PCA9539_GPIO_XFP_2_TX_FAULT_N  (SPX70D0_082F_I2C_PCA9539_1_BASE + SPX70D0_082F_PCA953x_GPIO(0,5))
#define SPX70D0_082F_PCA9539_GPIO_XFP_2_PRSNT_N     (SPX70D0_082F_I2C_PCA9539_1_BASE + SPX70D0_082F_PCA953x_GPIO(0,6))
#define SPX70D0_082F_PCA9539_GPIO_XFP_3_TX_DIS_N    (SPX70D0_082F_I2C_PCA9539_1_BASE + SPX70D0_082F_PCA953x_GPIO(1,0))
#define SPX70D0_082F_PCA9539_GPIO_XFP_3_TX_FAULT_N  (SPX70D0_082F_I2C_PCA9539_1_BASE + SPX70D0_082F_PCA953x_GPIO(1,1))
#define SPX70D0_082F_PCA9539_GPIO_XFP_3_PRSNT_N     (SPX70D0_082F_I2C_PCA9539_1_BASE + SPX70D0_082F_PCA953x_GPIO(1,2))
#define SPX70D0_082F_PCA9539_GPIO_XFP_4_TX_DIS_N    (SPX70D0_082F_I2C_PCA9539_1_BASE + SPX70D0_082F_PCA953x_GPIO(1,4))
#define SPX70D0_082F_PCA9539_GPIO_XFP_4_TX_FAULT_N  (SPX70D0_082F_I2C_PCA9539_1_BASE + SPX70D0_082F_PCA953x_GPIO(1,5))
#define SPX70D0_082F_PCA9539_GPIO_XFP_4_PRSNT_N     (SPX70D0_082F_I2C_PCA9539_1_BASE + SPX70D0_082F_PCA953x_GPIO(1,6))

#define SPX70D0_082F_PCA9539_GPIO_XFP_5_TX_DIS_N    (SPX70D0_082F_I2C_PCA9539_4_BASE + SPX70D0_082F_PCA953x_GPIO(0,0))
#define SPX70D0_082F_PCA9539_GPIO_XFP_5_TX_FAULT_N  (SPX70D0_082F_I2C_PCA9539_4_BASE + SPX70D0_082F_PCA953x_GPIO(0,1))
#define SPX70D0_082F_PCA9539_GPIO_XFP_5_PRSNT_N     (SPX70D0_082F_I2C_PCA9539_4_BASE + SPX70D0_082F_PCA953x_GPIO(0,2))
#define SPX70D0_082F_PCA9539_GPIO_XFP_6_TX_DIS_N    (SPX70D0_082F_I2C_PCA9539_4_BASE + SPX70D0_082F_PCA953x_GPIO(0,4))
#define SPX70D0_082F_PCA9539_GPIO_XFP_6_TX_FAULT_N  (SPX70D0_082F_I2C_PCA9539_4_BASE + SPX70D0_082F_PCA953x_GPIO(0,5))
#define SPX70D0_082F_PCA9539_GPIO_XFP_6_PRSNT_N     (SPX70D0_082F_I2C_PCA9539_4_BASE + SPX70D0_082F_PCA953x_GPIO(0,6))
#define SPX70D0_082F_PCA9539_GPIO_XFP_7_TX_DIS_N    (SPX70D0_082F_I2C_PCA9539_4_BASE + SPX70D0_082F_PCA953x_GPIO(1,0))
#define SPX70D0_082F_PCA9539_GPIO_XFP_7_TX_FAULT_N  (SPX70D0_082F_I2C_PCA9539_4_BASE + SPX70D0_082F_PCA953x_GPIO(1,1))
#define SPX70D0_082F_PCA9539_GPIO_XFP_7_PRSNT_N     (SPX70D0_082F_I2C_PCA9539_4_BASE + SPX70D0_082F_PCA953x_GPIO(1,2))
#define SPX70D0_082F_PCA9539_GPIO_XFP_8_TX_DIS_N    (SPX70D0_082F_I2C_PCA9539_4_BASE + SPX70D0_082F_PCA953x_GPIO(1,4))
#define SPX70D0_082F_PCA9539_GPIO_XFP_8_TX_FAULT_N  (SPX70D0_082F_I2C_PCA9539_4_BASE + SPX70D0_082F_PCA953x_GPIO(1,5))
#define SPX70D0_082F_PCA9539_GPIO_XFP_8_PRSNT_N     (SPX70D0_082F_I2C_PCA9539_4_BASE + SPX70D0_082F_PCA953x_GPIO(1,6))

#define SPX70D0_082F_PCA9539_GPIO_QSFP28_1_LPMODE_N (SPX70D0_082F_I2C_PCA9539_3_BASE + SPX70D0_082F_PCA953x_GPIO(0,0))
#define SPX70D0_082F_PCA9539_GPIO_QSFP28_1_INT_N    (SPX70D0_082F_I2C_PCA9539_3_BASE + SPX70D0_082F_PCA953x_GPIO(0,1))
#define SPX70D0_082F_PCA9539_GPIO_QSFP28_1_PRSNT_N  (SPX70D0_082F_I2C_PCA9539_3_BASE + SPX70D0_082F_PCA953x_GPIO(0,2))
#define SPX70D0_082F_PCA9539_GPIO_QSFP28_1_RESET_N  (SPX70D0_082F_I2C_PCA9539_3_BASE + SPX70D0_082F_PCA953x_GPIO(0,3))

#define SPX70D0_082F_PCA9539_GPIO_QSFP28_2_LPMODE_N (SPX70D0_082F_I2C_PCA9539_3_BASE + SPX70D0_082F_PCA953x_GPIO(1,0))
#define SPX70D0_082F_PCA9539_GPIO_QSFP28_2_INT_N    (SPX70D0_082F_I2C_PCA9539_3_BASE + SPX70D0_082F_PCA953x_GPIO(1,1))
#define SPX70D0_082F_PCA9539_GPIO_QSFP28_2_PRSNT_N  (SPX70D0_082F_I2C_PCA9539_3_BASE + SPX70D0_082F_PCA953x_GPIO(1,2))
#define SPX70D0_082F_PCA9539_GPIO_QSFP28_2_RESET_N  (SPX70D0_082F_I2C_PCA9539_3_BASE + SPX70D0_082F_PCA953x_GPIO(1,3))


#endif /* __X86_64_ALPHANETWORKS_SPX70D0_082F_GPIO_TABLE_H__ */
