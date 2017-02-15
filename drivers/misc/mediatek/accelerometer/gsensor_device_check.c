#include "gsensor_device_check.h"
#include <linux/module.h>
#include <linux/hwmsen_dev.h>


//sensor name used only in algorithm
char SENSOR_NAME[20]="none";
#define MAX_CHOOSE_G_NUM 2 //number of using files,actually should be same with sensor number

//detail information of sensors : start
gsensor_sensor_info gsensor_sensor_list[MULTI_SENSOR_NUM]={
	{  //sensor1
	    "BOSCH", /* sensor company */
	    "BMA255",    /* sensor name */
	    0x10,      /* slave_address */
	    0x00,      /* who_am_I_reg */
	    0xFA,      /* who_am_I_value */
	},
	{  //sensor2
	   "STM",  /* sensor company */
	   "K303B-ACC",     /* sensor name */
	   0x1D,       /* slave_address */
	   0x0F,       /* who_am_I_reg */
	   0x41,       /* who_am_I_value */
	},
};

enum sensor_list{
      BOSCH,
      STM
};
//detail information of sensors : end

struct i2c_msg msg[1];
static int Is_Sensor_Checked = 0;

int Gsensor_Dev_Check(void)
{
	struct i2c_adapter *adap;
	unsigned char data[2]={0};
	int err = 0;
    int i = 0;	

   SEN_FUN();

   if(Is_Sensor_Checked==0)
   {
	adap = i2c_get_adapter(GSENSOR_I2C_BUS_NUM);
	if (!adap)
		return -ENODEV;
		
	for(i=0;i<MULTI_SENSOR_NUM;i++)
	{
		data[0] = gsensor_sensor_list[i].who_am_I_reg;	
		msg->addr = gsensor_sensor_list[i].slave_address;	
		msg->flags = 0;
		msg->len = 1;
		msg->buf = data;
		msg->timing=100;		
		err = i2c_transfer(adap, msg, 1);

		msg->addr =  gsensor_sensor_list[i].slave_address;
		msg->flags = I2C_M_RD;
		msg->len = 1;
		msg->buf = data;
		err = i2c_transfer(adap, msg, 1);

	   if(err==1)
		 {
			if(data[0]==gsensor_sensor_list[i].who_am_I_value)
			{
			   SEN_LOG("Gsensor_Dev_Check: Read Value = 0x%x\n",data[0]);
			   SEN_LOG("Gsensor_Dev_Check : %s Sensor\n",gsensor_sensor_list[i].name);
			   strcpy(SENSOR_NAME,gsensor_sensor_list[i].name); 
               break;			   
            }			   
		 }
  
    }

    Is_Sensor_Checked = 1; //check only once       
     return err;
   	}
   else
   	{
        SEN_LOG("Gsensor_Dev_Check : %s Sensor(Checked)\n",SENSOR_NAME);
		err=1;
        return err;
   	}
}

extern struct sensor_init_info* gsensor_init_list[];
int Gsensor_Dev_Register(void)
{
	int err = -1;
	int i=0;

	for(i = 0; i < MAX_CHOOSE_G_NUM; i++)
	{
	  SEN_LOG("Gsensor_Dev_Register : i=%d\n",i);

	   if(!strcmp(SENSOR_NAME,gsensor_init_list[i]->name))
	    err = gsensor_init_list[i]->init();
	   
		if(0 == err)
		{
		   SEN_LOG("gsensor %s probe ok\n", SENSOR_NAME);
	       return err;
		}
	}

	 if(err !=0)
		{
			 SEN_LOG("Gsensor_Dev_Register : re-check all drivers\n");
				for(i = 0; i < MAX_CHOOSE_G_NUM; i++)
				{
				  SEN_LOG(" i=%d\n",i);
				  if(0 != gsensor_init_list[i])
				    {
					    err = gsensor_init_list[i]->init();
						if(0 == err)
						{
						   SEN_LOG(" gsensor %s probe ok\n", SENSOR_NAME);
						   break;
						}
				    }
				}
	     }  
	 
	if(i == MULTI_SENSOR_NUM)
	{
	   SEN_LOG("gsensor probe fail\n");
	}

	return err;
}

int Gsensor_Probe_Available(struct i2c_client *client)
{
   int result = 0;  //result 1 : doing probe
   int i=0;

   if(Is_Sensor_Checked==0)
   {
     SEN_LOG("Gsensor_Probe_Available : Is_Sensor_Checked==0\n");	
     result=1;   
   }
   else
   {
     SEN_LOG("Gsensor_Probe_Available : Is_Sensor_Checked==1\n");	   
      for(i=0;i<MULTI_SENSOR_NUM;i++)
	   {
	    if(gsensor_sensor_list[i].slave_address==client->addr)
	    	{
		     if(!strcmp(SENSOR_NAME,gsensor_sensor_list[i].name))
		        {
	              SEN_LOG("TEST8 : strcmp is success\n");	   	
		          SEN_LOG("i2c = 0x%x, name = %s\n",client->addr,SENSOR_NAME);
				  result=1;
				}
	    	}
	   } 
   }
  return result;
}
/* End of file */
