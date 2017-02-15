#include "msensor_device_check.h"
#include <linux/module.h>
#include <linux/hwmsen_dev.h>


//sensor name used only in algorithm
char GSENSOR_NAME[20]="none";
#define MAX_CHOOSE_M_NUM 2 //number of using files,actually should be same with sensor number

//detail information of sensors : start
msensor_sensor_info msensor_sensor_list[MULTI_SENSOR_NUM]={
	{  //sensor1
	    "BOSCH", /* sensor company */
	    "BMM050",    /* sensor name */
	    0x12,      /* slave_address */
	    0x40,      /* who_am_I_reg */
	    0x32,      /* who_am_I_value */
	},
	{  //sensor2
	   "STM",  /* sensor company */
	   "K303B",     /* sensor name */
	   0x1E,       /* slave_address */
	   0x0F,       /* who_am_I_reg */
	   0x3D,       /* who_am_I_value */
	},
};

enum sensor_list{
      BOSCH,
      STM
};
//detail information of sensors : end

struct i2c_msg mag_msg[1];
static int Is_MSensor_Checked = 0;

int Msensor_Dev_Check(void)
{
	struct i2c_adapter *adap;
	unsigned char data[2]={0};
	int err = 0;
    int i = 0;	

   SEN_FUN();

   if(Is_MSensor_Checked==0)
   {
	adap = i2c_get_adapter(MSENSOR_I2C_BUS_NUM);
	if (!adap)
		return -ENODEV;
		
	for(i=0;i<MULTI_SENSOR_NUM;i++)
	{
		data[0] = msensor_sensor_list[i].who_am_I_reg;	
		mag_msg->addr = msensor_sensor_list[i].slave_address;	
		mag_msg->flags = 0;
		mag_msg->len = 1;
		mag_msg->buf = data;
		mag_msg->timing=100;		
		err = i2c_transfer(adap, mag_msg, 1);

		mag_msg->addr =  msensor_sensor_list[i].slave_address;
		mag_msg->flags = I2C_M_RD;
		mag_msg->len = 1;
		mag_msg->buf = data;
		err = i2c_transfer(adap, mag_msg, 1);

	   if(err==1)
		 {
			if(data[0]==msensor_sensor_list[i].who_am_I_value)
			{
			   SEN_LOG("Msensor_Dev_Check: Read Value = 0x%x\n",data[0]);
			   SEN_LOG("Msensor_Dev_Check : %s Sensor\n",msensor_sensor_list[i].name);
			   strcpy(GSENSOR_NAME,msensor_sensor_list[i].name); 
               break;			   
            }			   
		 }
  
    }

    Is_MSensor_Checked = 1; //check only once       
     return err;
   	}
   else
   	{
        SEN_LOG("Msensor_Dev_Check : %s Sensor(Checked)\n",GSENSOR_NAME);
		err=1;
        return err;
   	}
}

extern struct sensor_init_info* msensor_init_list[];
int Msensor_Dev_Register(void)
{
	int err = -1;
	int i=0;

	for(i = 0; i < MAX_CHOOSE_M_NUM; i++)
	{
	  SEN_LOG("Msensor_Dev_Register : i=%d\n",i);

	   if(!strcmp(GSENSOR_NAME,msensor_init_list[i]->name))
	    err = msensor_init_list[i]->init();
	   
		if(0 == err)
		{
		   SEN_LOG("msensor %s probe ok\n", GSENSOR_NAME);
	       return err;
		}
	}

	 if(err !=0)
		{
			 SEN_LOG("Msensor_Dev_Register : re-check all drivers\n");
				for(i = 0; i < MAX_CHOOSE_M_NUM; i++)
				{
				  SEN_LOG(" i=%d\n",i);
				  if(0 != msensor_init_list[i])
				    {
					    err = msensor_init_list[i]->init();
						if(0 == err)
						{
						   SEN_LOG(" msensor %s probe ok\n", GSENSOR_NAME);
						   break;
						}
				    }
				}
	     }  
	 
	if(i == MULTI_SENSOR_NUM)
	{
	   SEN_LOG("msensor probe fail\n");
	}

	return err;
}

int Msensor_Probe_Available(struct i2c_client *client)
{
   int result = 0;  //result 1 : doing probe
   int i=0;

   if(Is_MSensor_Checked==0)
   {
     SEN_LOG("Msensor_Probe_Available : Is_MSensor_Checked==0\n");	
     result=1;   
   }
   else
   {
     SEN_LOG("Msensor_Probe_Available : Is_MSensor_Checked==1\n");	   
      for(i=0;i<MULTI_SENSOR_NUM;i++)
	   {
	    if(msensor_sensor_list[i].slave_address==client->addr)
	    	{
		     if(!strcmp(GSENSOR_NAME,msensor_sensor_list[i].name))
		        {
	              SEN_LOG("TEST8 : strcmp is success\n");	   	
		          SEN_LOG("i2c = 0x%x, name = %s\n",client->addr,GSENSOR_NAME);
				  result=1;
				}
	    	}
	   } 
   }
  return result;
}
/* End of file */
