#include "alsps_device_check.h"
#include <linux/module.h>
#include <linux/hwmsen_dev.h>


//sensor name used only in algorithm
char SENSOR_NAME[20]="none";
#define MAX_CHOOSE_G_NUM 2 //number of using files,actually should be same with sensor number

//detail information of sensors : start
alsps_sensor_info alsps_sensor_list[MULTI_SENSOR_NUM]={
	{  //sensor1
	    "AVAGO", /* sensor company */
	    "APDS9130",    /* sensor name */
	    0x39,      /* slave_address */
	    0x12,      /* who_am_I_reg */
	    0x39,      /* who_am_I_value */
	},
	{  //sensor2
	   "ROHM",  /* sensor company */
	   "RPR0521",     /* sensor name */
	   0x38,       /* slave_address */
	   0x92,       /* who_am_I_reg */
	   0xe0,       /* who_am_I_value */
	},
};

enum sensor_list{
      AVAGO,
      ROHM
};
//detail information of sensors : end

struct i2c_msg msg[1];
static int Is_Sensor_Checked = 0;

int Alsps_Dev_Check(void)
{
	struct i2c_adapter *adap;
	unsigned char data[2]={0};
	int err = 0;
    int i = 0;	

   SEN_FUN();

   if(Is_Sensor_Checked==0)
   {
	adap = i2c_get_adapter(ALSPS_I2C_BUS_NUM);
	if (!adap)
		return -ENODEV;
		
	for(i=0;i<MULTI_SENSOR_NUM;i++)
	{
		data[0] = alsps_sensor_list[i].who_am_I_reg;	
		msg->addr = alsps_sensor_list[i].slave_address;	
		msg->flags = 0;
		msg->len = 1;
		msg->buf = data;
		msg->timing=100;		
		err = i2c_transfer(adap, msg, 1);

		msg->addr =  alsps_sensor_list[i].slave_address;
		msg->flags = I2C_M_RD;
		msg->len = 1;
		msg->buf = data;
		err = i2c_transfer(adap, msg, 1);

	   if(err==1)
		 {
			if(data[0]==alsps_sensor_list[i].who_am_I_value)
			{
			   SEN_LOG("Alsps_Dev_Check: Read Value = 0x%x\n",data[0]);
			   SEN_LOG("Alsps_Dev_Check : %s Sensor\n",alsps_sensor_list[i].name);
			   strcpy(SENSOR_NAME,alsps_sensor_list[i].name); 
               break;			   
            }			   
		 }
  
    }
#if 0
 //if need, setting flag which want to use : start
    if(!strcmp(SENSOR_NAME,"K2HH"))
	{
	  K2HH=1;
	}
 //if need, setting flag which want to use : end
#endif
    Is_Sensor_Checked = 1; //check only once       
     return err;
   	}
   else
   	{
        SEN_LOG("Alsps_Dev_Check : %s Sensor(Checked)\n",SENSOR_NAME);
		err=1;
        return err;
   	}
}

extern struct sensor_init_info* alsps_init_list[];
int Alsps_Dev_Register(void)
{
	int err = -1;
	int i=0;

	for(i = 0; i < MAX_CHOOSE_G_NUM; i++)
	{
	  SEN_LOG("Alsps_Dev_Register : i=%d\n",i);

	   if(!strcmp(SENSOR_NAME,alsps_init_list[i]->name))
	    err = alsps_init_list[i]->init();
	   
		if(0 == err)
		{
		   SEN_LOG("alsps %s probe ok\n", SENSOR_NAME);
	       return err;
		}
	}

	 if(err !=0)
		{
			 SEN_LOG("Alsps_Dev_Register : re-check all drivers\n");
				for(i = 0; i < MAX_CHOOSE_G_NUM; i++)
				{
				  SEN_LOG(" i=%d\n",i);
				  if(0 != alsps_init_list[i])
				    {
					    err = alsps_init_list[i]->init();
						if(0 == err)
						{
						   SEN_LOG(" alsps %s probe ok\n", SENSOR_NAME);
						   break;
						}
				    }
				}
	     }  
	 
	if(i == MULTI_SENSOR_NUM)
	{
	   SEN_LOG("alsps probe fail\n");
	}

	return err;
}

int Alsps_Probe_Available(struct i2c_client *client)
{
   int result = 0;  //result 1 : doing probe
   int i=0;

   if(Is_Sensor_Checked==0)
   {
     SEN_LOG("Alsps_Probe_Available : Is_Sensor_Checked==0\n");	
     result=1;   
   }
   else
   {
     SEN_LOG("Alsps_Probe_Available : Is_Sensor_Checked==1\n");	   
      for(i=0;i<MULTI_SENSOR_NUM;i++)
	   {
	    if(alsps_sensor_list[i].slave_address==client->addr)
	    	{
		     if(!strcmp(SENSOR_NAME,alsps_sensor_list[i].name))
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
