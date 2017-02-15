#include <linux/i2c.h>
#include <linux/string.h>


//set i2c bus & number of sensors which to select : start
#define MULTI_SENSOR_NUM 3
#define MSENSOR_I2C_BUS_NUM 2
//set i2c bus & number of sensors which to select : end

//sensor basic information
typedef struct {
    char vendor[20];  /* sensor company */
    char name[20];    /* sensor name */
    int slave_address;  /* slave_address */
    int who_am_I_reg;   /* who_am_I_reg */
    int who_am_I_value;  /* who_am_I_value */
}msensor_sensor_info;

/*----------------------------------------------------------------------------*/
#define SEN_TAG                  "[DualMagSenor] "
#define SEN_FUN(f)               printk(KERN_INFO SEN_TAG"%s\n", __FUNCTION__)
#define SEN_ERR(fmt, args...)    printk(KERN_ERR SEN_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SEN_LOG(fmt, args...)    printk(KERN_INFO SEN_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/

extern int Msensor_Dev_Check(void);
extern int Msensor_Probe_Available(struct i2c_client *client);
extern int Msensor_Dev_Register(void);

/* END of file */
