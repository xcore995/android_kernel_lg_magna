
#if defined(CONFIG_MTK_THERMAL_PA_VIA_ATCMD)
extern int mtk_mdm_get_mdinfoex(int opcode, int *value);
extern int mtk_mdm_set_mdinfoex_threshold(int opcode, int threshold);

static DEFINE_MUTEX(TSMD_lock);

#define MTK_TS_PA_GET_MD_NTC_TEMP   (1)
#define MTK_TS_PA_GET_MD_DRAM_TEMP  (1)
#define MTK_TS_PA_GET_PA_NTC_TEMP   (1)

#if MTK_TS_PA_GET_MD_NTC_TEMP
static unsigned int ntc6290_interval = 0; /* ms */
static struct thermal_zone_device *ntc6290_thz_dev;
static int ntc6290_THERMAL_TRIP[10] = {0,0,0,0,0,0,0,0,0,0};
static unsigned int ntc6290_trip_temp[10] = {115000,105000,95000,85000,75000,65000,55000,45000,35000,25000};
static int ntc6290_kernelmode = 0;
static int ntc6290_num_trip=1;
static char ntc6290_bind0[20]={0};
static char ntc6290_bind1[20]={0};
static char ntc6290_bind2[20]={0};
static char ntc6290_bind3[20]={0};
static char ntc6290_bind4[20]={0};
static char ntc6290_bind5[20]={0};
static char ntc6290_bind6[20]={0};
static char ntc6290_bind7[20]={0};
static char ntc6290_bind8[20]={0};
static char ntc6290_bind9[20]={0};

static int ntc6290_get_hw_temp(void)
{
	int value;
    mutex_lock(&TSMD_lock);	
	if (mtk_mdm_get_mdinfoex(0, &value) == 0)
	{
	    if (value == -32767)
	    {
	        mutex_unlock(&TSMD_lock);
	        return -127000;
	    }
	    else
	    {
	        mutex_unlock(&TSMD_lock);
    	    return value*1000;
        }
	}
	mutex_unlock(&TSMD_lock);
	return -127000;
}
    
static int ntc6290_get_temp(struct thermal_zone_device *thermal,
             unsigned long *t)
{
	*t = ntc6290_get_hw_temp();

    // set new threshold
    {
        int i = ntc6290_num_trip - 1;
        for (; i >= 0; i--)
        {
            if (ntc6290_trip_temp[i] > *t)
            {
                mtk_mdm_set_mdinfoex_threshold(0, ntc6290_trip_temp[i]/1000);
                break;
            }
        }
    }
	
	if (*t > 85000)
        mtktspa_dprintk("%s temp=%lu\n", __func__, *t);
	return 0;
}

static int ntc6290_bind(struct thermal_zone_device *thermal,
                        struct thermal_cooling_device *cdev)
{
	int table_val=0;

	if(!strcmp(cdev->type, ntc6290_bind0))
	{
		table_val = 0;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind1))
	{
		table_val = 1;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind2))
	{
		table_val = 2;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind3))
	{
		table_val = 3;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind4))
	{
		table_val = 4;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind5))
	{
		table_val = 5;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind6))
	{
		table_val = 6;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind7))
	{
		table_val = 7;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind8))
	{
		table_val = 8;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind9))
	{
		table_val = 9;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else
		return 0;


	if (mtk_thermal_zone_bind_cooling_device(thermal, table_val, cdev)) 
	{
		mtktspa_dprintk("[%s] error binding cooling dev\n", __func__);
		return -EINVAL;
	} 
	else 
	{
		mtktspa_dprintk("[%s] binding OK\n", __func__);
	}

	return 0;
}

static int ntc6290_unbind(struct thermal_zone_device *thermal,
        struct thermal_cooling_device *cdev)
{
	int table_val=0;

	if(!strcmp(cdev->type, ntc6290_bind0))
	{
		table_val = 0;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind1))
	{
		table_val = 1;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind2))
	{
		table_val = 2;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind3))
	{
		table_val = 3;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind4))
	{
		table_val = 4;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind5))
	{
		table_val = 5;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind6))
	{
		table_val = 6;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind7))
	{
		table_val = 7;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind8))
	{
		table_val = 8;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntc6290_bind9))
	{
		table_val = 9;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else
		return 0;

	if (thermal_zone_unbind_cooling_device(thermal, table_val, cdev)) 
	{
		mtktspa_dprintk("[%s] error unbinding cooling dev\n", __func__);
		return -EINVAL;
	} 
	else 
	{
		mtktspa_dprintk("[%s] unbinding OK\n", __func__);
	}

	return 0;
}

static int ntc6290_get_mode(struct thermal_zone_device *thermal,
          enum thermal_device_mode *mode)
{
	*mode = (ntc6290_kernelmode) ? THERMAL_DEVICE_ENABLED
		: THERMAL_DEVICE_DISABLED;

	return 0;
}

static int ntc6290_set_mode(struct thermal_zone_device *thermal,
          enum thermal_device_mode mode)
{
	ntc6290_kernelmode = mode;
	return 0;
}

static int ntc6290_get_trip_type(struct thermal_zone_device *thermal, int trip,
         enum thermal_trip_type *type)
{
	*type = ntc6290_THERMAL_TRIP[trip];
	return 0;
}

static int ntc6290_get_trip_temp(struct thermal_zone_device *thermal, int trip,
         unsigned long *temp)
{
	*temp = ntc6290_trip_temp[trip];
	return 0;
}

static int ntc6290_get_crit_temp(struct thermal_zone_device *thermal,
         unsigned long *temperature)
{
	*temperature = 115000;
	return 0;
}

/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops ntc6290_dev_ops = {
	.bind = ntc6290_bind,
	.unbind = ntc6290_unbind,
	.get_temp = ntc6290_get_temp,
	.get_mode = ntc6290_get_mode,
	.set_mode = ntc6290_set_mode,
	.get_trip_type = ntc6290_get_trip_type,
	.get_trip_temp = ntc6290_get_trip_temp,
	.get_crit_temp = ntc6290_get_crit_temp,
};

static int ntc6290_read(struct seq_file *m, void *v)
{
	seq_printf(m, "[ntc6290_read] \ntrip_0_temp=%d \ntrip_1_temp=%d \ntrip_2_temp=%d \ntrip_3_temp=%d \ntrip_4_temp=%d \n\
trip_5_temp=%d \ntrip_6_temp=%d \ntrip_7_temp=%d \ntrip_8_temp=%d \ntrip_9_temp=%d \n\
g_THERMAL_TRIP_0=%d \ng_THERMAL_TRIP_1=%d \ng_THERMAL_TRIP_2=%d \ng_THERMAL_TRIP_3=%d \ng_THERMAL_TRIP_4=%d \n\
g_THERMAL_TRIP_5=%d \ng_THERMAL_TRIP_6=%d \ng_THERMAL_TRIP_7=%d \ng_THERMAL_TRIP_8=%d \ng_THERMAL_TRIP_9=%d \n\
cooldev0=%s \ncooldev1=%s \ncooldev2=%s \ncooldev3=%s \ncooldev4=%s \n\
cooldev5=%s \ncooldev6=%s \ncooldev7=%s \ncooldev8=%s \ncooldev9=%s \ntime_ms=%d\n",
				ntc6290_trip_temp[0],ntc6290_trip_temp[1],ntc6290_trip_temp[2],ntc6290_trip_temp[3],ntc6290_trip_temp[4],
				ntc6290_trip_temp[5],ntc6290_trip_temp[6],ntc6290_trip_temp[7],ntc6290_trip_temp[8],ntc6290_trip_temp[9],
				ntc6290_THERMAL_TRIP[0],ntc6290_THERMAL_TRIP[1],ntc6290_THERMAL_TRIP[2],ntc6290_THERMAL_TRIP[3],ntc6290_THERMAL_TRIP[4],
				ntc6290_THERMAL_TRIP[5],ntc6290_THERMAL_TRIP[6],ntc6290_THERMAL_TRIP[7],ntc6290_THERMAL_TRIP[8],ntc6290_THERMAL_TRIP[9],
				ntc6290_bind0,ntc6290_bind1,ntc6290_bind2,ntc6290_bind3,ntc6290_bind4,ntc6290_bind5,ntc6290_bind6,ntc6290_bind7,ntc6290_bind8,ntc6290_bind9,
				ntc6290_interval);
    
	return 0;
}

static ssize_t ntc6290_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	int len=0,time_msec=0;
	int trip[10]={0};
	int t_type[10]={0};
	int i;
	char bind0[20],bind1[20],bind2[20],bind3[20],bind4[20];
	char bind5[20],bind6[20],bind7[20],bind8[20],bind9[20];
	char desc[512];


	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';
		
	if (sscanf(desc, "%d %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d",
				&ntc6290_num_trip, &trip[0],&t_type[0],bind0, &trip[1],&t_type[1],bind1,
				&trip[2],&t_type[2],bind2, &trip[3],&t_type[3],bind3,
				&trip[4],&t_type[4],bind4, &trip[5],&t_type[5],bind5,
				&trip[6],&t_type[6],bind6, &trip[7],&t_type[7],bind7,
				&trip[8],&t_type[8],bind8, &trip[9],&t_type[9],bind9,
				&time_msec) == 32)
	{
		mtktspa_dprintk("[ntc6290_write] unregister_thermal\n");
		if (ntc6290_thz_dev) 
    	{
    		mtk_thermal_zone_device_unregister(ntc6290_thz_dev);
    		ntc6290_thz_dev = NULL;
    	}
	
		for(i=0; i<ntc6290_num_trip; i++)
			ntc6290_THERMAL_TRIP[i] = t_type[i];	

		ntc6290_bind0[0]=ntc6290_bind1[0]=ntc6290_bind2[0]=ntc6290_bind3[0]=ntc6290_bind4[0]=ntc6290_bind5[0]=ntc6290_bind6[0]=ntc6290_bind7[0]=ntc6290_bind8[0]=ntc6290_bind9[0]='\0';
				
		for(i=0; i<20; i++)
		{
			ntc6290_bind0[i]=bind0[i]; 
			ntc6290_bind1[i]=bind1[i]; 
			ntc6290_bind2[i]=bind2[i]; 
			ntc6290_bind3[i]=bind3[i]; 
			ntc6290_bind4[i]=bind4[i];
			ntc6290_bind5[i]=bind5[i]; 
			ntc6290_bind6[i]=bind6[i]; 
			ntc6290_bind7[i]=bind7[i]; 
			ntc6290_bind8[i]=bind8[i]; 
			ntc6290_bind9[i]=bind9[i];
		}

		mtktspa_dprintk("[ntc6290_write] g_THERMAL_TRIP_0=%d,g_THERMAL_TRIP_1=%d,g_THERMAL_TRIP_2=%d,g_THERMAL_TRIP_3=%d,g_THERMAL_TRIP_4=%d,\
g_THERMAL_TRIP_5=%d,g_THERMAL_TRIP_6=%d,g_THERMAL_TRIP_7=%d,g_THERMAL_TRIP_8=%d,g_THERMAL_TRIP_9=%d,\n",
				ntc6290_THERMAL_TRIP[0],ntc6290_THERMAL_TRIP[1],ntc6290_THERMAL_TRIP[2],ntc6290_THERMAL_TRIP[3],ntc6290_THERMAL_TRIP[4],
				ntc6290_THERMAL_TRIP[5],ntc6290_THERMAL_TRIP[6],ntc6290_THERMAL_TRIP[7],ntc6290_THERMAL_TRIP[8],ntc6290_THERMAL_TRIP[9]);
	mtktspa_dprintk("[ntc6290_write] cooldev0=%s,cooldev1=%s,cooldev2=%s,cooldev3=%s,cooldev4=%s,\
cooldev5=%s,cooldev6=%s,cooldev7=%s,cooldev8=%s,cooldev9=%s\n",
				ntc6290_bind0,ntc6290_bind1,ntc6290_bind2,ntc6290_bind3,ntc6290_bind4,ntc6290_bind5,ntc6290_bind6,ntc6290_bind7,ntc6290_bind8,ntc6290_bind9);

		for(i=0; i<ntc6290_num_trip; i++)
		{
			ntc6290_trip_temp[i]=trip[i];
		}

		ntc6290_interval=time_msec;

		mtktspa_dprintk("[ntc6290_write] trip_0_temp=%d,trip_1_temp=%d,trip_2_temp=%d,trip_3_temp=%d,trip_4_temp=%d,\
trip_5_temp=%d,trip_6_temp=%d,trip_7_temp=%d,trip_8_temp=%d,trip_9_temp=%d,time_ms=%d\n", 
				ntc6290_trip_temp[0],ntc6290_trip_temp[1],ntc6290_trip_temp[2],ntc6290_trip_temp[3],ntc6290_trip_temp[4],
				ntc6290_trip_temp[5],ntc6290_trip_temp[6],ntc6290_trip_temp[7],ntc6290_trip_temp[8],ntc6290_trip_temp[9],ntc6290_interval);

		mtktspa_dprintk("[ntc6290_write] register_thermal\n");
		ntc6290_thz_dev = mtk_thermal_zone_device_register("ntcmd", ntc6290_num_trip, NULL,
		    &ntc6290_dev_ops, 0, 0, 0, ntc6290_interval);

		return count;
	}
	else
	{
		mtktspa_dprintk("[ntc6290_write] bad argument\n");
	}
		
	return -EINVAL;
		
}

static int tzmdntc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ntc6290_read, NULL);
}

static const struct file_operations tzmdntc_fops = {
    .owner = THIS_MODULE,
    .open = tzmdntc_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = ntc6290_write,
    .release = single_release,
};
#endif

#if MTK_TS_PA_GET_MD_DRAM_TEMP
static unsigned int dram6290_interval = 0; /* ms */
static struct thermal_zone_device *dram6290_thz_dev;
static int dram6290_THERMAL_TRIP[10] = {0,0,0,0,0,0,0,0,0,0};
static unsigned int dram6290_trip_temp[10] = {115000,105000,95000,85000,75000,65000,55000,45000,35000,25000};
static int dram6290_kernelmode = 0;
static int dram6290_num_trip=1;
static char dram6290_bind0[20]={0};
static char dram6290_bind1[20]={0};
static char dram6290_bind2[20]={0};
static char dram6290_bind3[20]={0};
static char dram6290_bind4[20]={0};
static char dram6290_bind5[20]={0};
static char dram6290_bind6[20]={0};
static char dram6290_bind7[20]={0};
static char dram6290_bind8[20]={0};
static char dram6290_bind9[20]={0};

static int dram6290_get_hw_temp(void)
{
	int value;
	
    mutex_lock(&TSMD_lock);	
	if (mtk_mdm_get_mdinfoex(1, &value) == 0)
	{
	    if (value == -32767)
	    {
	        mutex_unlock(&TSMD_lock);
	        return -127000;
	    }
	    else
	    {
	        mutex_unlock(&TSMD_lock);
	        if (value > 3)
	            mtktspa_dprintk("%s value=%d\n", __func__, value);
	        return value*1000;
	    }
#if 0
	    else if (value <= 3)
	    {
	        mutex_unlock(&TSMD_lock);
	        return 85000;
    	}
    	else if (value > 3 && value < 7)
    	{
    	    mutex_unlock(&TSMD_lock);
    	    mtktspa_dprintk("%s value=%d\n", __func__, value);
    	    return 100000;
    	}
    	else
    	{
    	    mutex_unlock(&TSMD_lock);
    	    mtktspa_dprintk("%s value=%d\n", __func__, value);
    	    return 125000;
    	}
#endif
	}
	mutex_unlock(&TSMD_lock);
	return -127000;
}
    
static int dram6290_get_temp(struct thermal_zone_device *thermal,
             unsigned long *t)
{
	*t = dram6290_get_hw_temp();

	// set new threshold
    {
        int i = dram6290_num_trip - 1;
        for (; i >= 0; i--)
        {
            if (dram6290_trip_temp[i] > *t)
            {
                mtk_mdm_set_mdinfoex_threshold(1, dram6290_trip_temp[i]/1000);
                break;
            }
        }
    }
    
	return 0;
}

static int dram6290_bind(struct thermal_zone_device *thermal,
                        struct thermal_cooling_device *cdev)
{
	int table_val=0;

	if(!strcmp(cdev->type, dram6290_bind0))
	{
		table_val = 0;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind1))
	{
		table_val = 1;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind2))
	{
		table_val = 2;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind3))
	{
		table_val = 3;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind4))
	{
		table_val = 4;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind5))
	{
		table_val = 5;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind6))
	{
		table_val = 6;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind7))
	{
		table_val = 7;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind8))
	{
		table_val = 8;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind9))
	{
		table_val = 9;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else
		return 0;


	if (mtk_thermal_zone_bind_cooling_device(thermal, table_val, cdev)) 
	{
		mtktspa_dprintk("[%s] error binding cooling dev\n", __func__);
		return -EINVAL;
	} 
	else 
	{
		mtktspa_dprintk("[%s] binding OK\n", __func__);
	}

	return 0;
}

static int dram6290_unbind(struct thermal_zone_device *thermal,
        struct thermal_cooling_device *cdev)
{
	int table_val=0;

	if(!strcmp(cdev->type, dram6290_bind0))
	{
		table_val = 0;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind1))
	{
		table_val = 1;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind2))
	{
		table_val = 2;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind3))
	{
		table_val = 3;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind4))
	{
		table_val = 4;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind5))
	{
		table_val = 5;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind6))
	{
		table_val = 6;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind7))
	{
		table_val = 7;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind8))
	{
		table_val = 8;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, dram6290_bind9))
	{
		table_val = 9;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else
		return 0;

	if (thermal_zone_unbind_cooling_device(thermal, table_val, cdev)) 
	{
		mtktspa_dprintk("[%s] error unbinding cooling dev\n", __func__);
		return -EINVAL;
	} 
	else 
	{
		mtktspa_dprintk("[%s] unbinding OK\n", __func__);
	}

	return 0;
}

static int dram6290_get_mode(struct thermal_zone_device *thermal,
          enum thermal_device_mode *mode)
{
	*mode = (dram6290_kernelmode) ? THERMAL_DEVICE_ENABLED
		: THERMAL_DEVICE_DISABLED;

	return 0;
}

static int dram6290_set_mode(struct thermal_zone_device *thermal,
          enum thermal_device_mode mode)
{
	dram6290_kernelmode = mode;
	return 0;
}

static int dram6290_get_trip_type(struct thermal_zone_device *thermal, int trip,
         enum thermal_trip_type *type)
{
	*type = dram6290_THERMAL_TRIP[trip];
	return 0;
}

static int dram6290_get_trip_temp(struct thermal_zone_device *thermal, int trip,
         unsigned long *temp)
{
	*temp = dram6290_trip_temp[trip];
	return 0;
}

static int dram6290_get_crit_temp(struct thermal_zone_device *thermal,
         unsigned long *temperature)
{
	*temperature = 115000;
	return 0;
}

/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops dram6290_dev_ops = {
	.bind = dram6290_bind,
	.unbind = dram6290_unbind,
	.get_temp = dram6290_get_temp,
	.get_mode = dram6290_get_mode,
	.set_mode = dram6290_set_mode,
	.get_trip_type = dram6290_get_trip_type,
	.get_trip_temp = dram6290_get_trip_temp,
	.get_crit_temp = dram6290_get_crit_temp,
};

static int dram6290_read(struct seq_file *m, void *v)
{
	seq_printf(m, "[dram6290_read] \ntrip_0_temp=%d \ntrip_1_temp=%d \ntrip_2_temp=%d \ntrip_3_temp=%d \ntrip_4_temp=%d \n\
trip_5_temp=%d \ntrip_6_temp=%d \ntrip_7_temp=%d \ntrip_8_temp=%d \ntrip_9_temp=%d \n\
g_THERMAL_TRIP_0=%d \ng_THERMAL_TRIP_1=%d \ng_THERMAL_TRIP_2=%d \ng_THERMAL_TRIP_3=%d \ng_THERMAL_TRIP_4=%d \n\
g_THERMAL_TRIP_5=%d \ng_THERMAL_TRIP_6=%d \ng_THERMAL_TRIP_7=%d \ng_THERMAL_TRIP_8=%d \ng_THERMAL_TRIP_9=%d \n\
cooldev0=%s \ncooldev1=%s \ncooldev2=%s \ncooldev3=%s \ncooldev4=%s \n\
cooldev5=%s \ncooldev6=%s \ncooldev7=%s \ncooldev8=%s \ncooldev9=%s \ntime_ms=%d\n",
				dram6290_trip_temp[0],dram6290_trip_temp[1],dram6290_trip_temp[2],dram6290_trip_temp[3],dram6290_trip_temp[4],
				dram6290_trip_temp[5],dram6290_trip_temp[6],dram6290_trip_temp[7],dram6290_trip_temp[8],dram6290_trip_temp[9],
				dram6290_THERMAL_TRIP[0],dram6290_THERMAL_TRIP[1],dram6290_THERMAL_TRIP[2],dram6290_THERMAL_TRIP[3],dram6290_THERMAL_TRIP[4],
				dram6290_THERMAL_TRIP[5],dram6290_THERMAL_TRIP[6],dram6290_THERMAL_TRIP[7],dram6290_THERMAL_TRIP[8],dram6290_THERMAL_TRIP[9],
				dram6290_bind0,dram6290_bind1,dram6290_bind2,dram6290_bind3,dram6290_bind4,dram6290_bind5,dram6290_bind6,dram6290_bind7,dram6290_bind8,dram6290_bind9,
				dram6290_interval);
    
	return 0;
}

static ssize_t dram6290_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	int len=0,time_msec=0;
	int trip[10]={0};
	int t_type[10]={0};
	int i;
	char bind0[20],bind1[20],bind2[20],bind3[20],bind4[20];
	char bind5[20],bind6[20],bind7[20],bind8[20],bind9[20];
	char desc[512];


	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';
		
	if (sscanf(desc, "%d %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d",
				&dram6290_num_trip, &trip[0],&t_type[0],bind0, &trip[1],&t_type[1],bind1,
				&trip[2],&t_type[2],bind2, &trip[3],&t_type[3],bind3,
				&trip[4],&t_type[4],bind4, &trip[5],&t_type[5],bind5,
				&trip[6],&t_type[6],bind6, &trip[7],&t_type[7],bind7,
				&trip[8],&t_type[8],bind8, &trip[9],&t_type[9],bind9,
				&time_msec) == 32)
	{
		mtktspa_dprintk("[dram6290_write] unregister_thermal\n");
		if (dram6290_thz_dev) 
    	{
    		mtk_thermal_zone_device_unregister(dram6290_thz_dev);
    		dram6290_thz_dev = NULL;
    	}
	
		for(i=0; i<dram6290_num_trip; i++)
			dram6290_THERMAL_TRIP[i] = t_type[i];	

		dram6290_bind0[0]=dram6290_bind1[0]=dram6290_bind2[0]=dram6290_bind3[0]=dram6290_bind4[0]=dram6290_bind5[0]=dram6290_bind6[0]=dram6290_bind7[0]=dram6290_bind8[0]=dram6290_bind9[0]='\0';
				
		for(i=0; i<20; i++)
		{
			dram6290_bind0[i]=bind0[i]; 
			dram6290_bind1[i]=bind1[i]; 
			dram6290_bind2[i]=bind2[i]; 
			dram6290_bind3[i]=bind3[i]; 
			dram6290_bind4[i]=bind4[i];
			dram6290_bind5[i]=bind5[i]; 
			dram6290_bind6[i]=bind6[i]; 
			dram6290_bind7[i]=bind7[i]; 
			dram6290_bind8[i]=bind8[i]; 
			dram6290_bind9[i]=bind9[i];
		}

		mtktspa_dprintk("[dram6290_write] g_THERMAL_TRIP_0=%d,g_THERMAL_TRIP_1=%d,g_THERMAL_TRIP_2=%d,g_THERMAL_TRIP_3=%d,g_THERMAL_TRIP_4=%d,\
g_THERMAL_TRIP_5=%d,g_THERMAL_TRIP_6=%d,g_THERMAL_TRIP_7=%d,g_THERMAL_TRIP_8=%d,g_THERMAL_TRIP_9=%d,\n",
				dram6290_THERMAL_TRIP[0],dram6290_THERMAL_TRIP[1],dram6290_THERMAL_TRIP[2],dram6290_THERMAL_TRIP[3],dram6290_THERMAL_TRIP[4],
				dram6290_THERMAL_TRIP[5],dram6290_THERMAL_TRIP[6],dram6290_THERMAL_TRIP[7],dram6290_THERMAL_TRIP[8],dram6290_THERMAL_TRIP[9]);
	mtktspa_dprintk("[dram6290_write] cooldev0=%s,cooldev1=%s,cooldev2=%s,cooldev3=%s,cooldev4=%s,\
cooldev5=%s,cooldev6=%s,cooldev7=%s,cooldev8=%s,cooldev9=%s\n",
				dram6290_bind0,dram6290_bind1,dram6290_bind2,dram6290_bind3,dram6290_bind4,dram6290_bind5,dram6290_bind6,dram6290_bind7,dram6290_bind8,dram6290_bind9);

		for(i=0; i<dram6290_num_trip; i++)
		{
			dram6290_trip_temp[i]=trip[i];
		}

		dram6290_interval=time_msec;

		mtktspa_dprintk("[dram6290_write] trip_0_temp=%d,trip_1_temp=%d,trip_2_temp=%d,trip_3_temp=%d,trip_4_temp=%d,\
trip_5_temp=%d,trip_6_temp=%d,trip_7_temp=%d,trip_8_temp=%d,trip_9_temp=%d,time_ms=%d\n", 
				dram6290_trip_temp[0],dram6290_trip_temp[1],dram6290_trip_temp[2],dram6290_trip_temp[3],dram6290_trip_temp[4],
				dram6290_trip_temp[5],dram6290_trip_temp[6],dram6290_trip_temp[7],dram6290_trip_temp[8],dram6290_trip_temp[9],dram6290_interval);

		mtktspa_dprintk("[dram6290_write] register_thermal\n");
		dram6290_thz_dev = mtk_thermal_zone_device_register("drammd", dram6290_num_trip, NULL,
		    &dram6290_dev_ops, 0, 0, 0, dram6290_interval);

		return count;
	}
	else
	{
		mtktspa_dprintk("[dram6290_write] bad argument\n");
	}
		
	return -EINVAL;
		
}

static int tzmddram_open(struct inode *inode, struct file *file)
{
    return single_open(file, dram6290_read, NULL);
}

static const struct file_operations tzmddram_fops = {
    .owner = THIS_MODULE,
    .open = tzmddram_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = dram6290_write,
    .release = single_release,
};
#endif

#if MTK_TS_PA_GET_PA_NTC_TEMP
static unsigned int ntcpa_interval = 0; /* ms */
static struct thermal_zone_device *ntcpa_thz_dev;
static int ntcpa_THERMAL_TRIP[10] = {0,0,0,0,0,0,0,0,0,0};
static unsigned int ntcpa_trip_temp[10] = {115000,105000,95000,85000,75000,65000,55000,45000,35000,25000};
static int ntcpa_kernelmode = 0;
static int ntcpa_num_trip=1;
static char ntcpa_bind0[20]={0};
static char ntcpa_bind1[20]={0};
static char ntcpa_bind2[20]={0};
static char ntcpa_bind3[20]={0};
static char ntcpa_bind4[20]={0};
static char ntcpa_bind5[20]={0};
static char ntcpa_bind6[20]={0};
static char ntcpa_bind7[20]={0};
static char ntcpa_bind8[20]={0};
static char ntcpa_bind9[20]={0};

static int ntcpa_get_hw_temp(void)
{
	int value;
    mutex_lock(&TSMD_lock);	
	if (mtk_mdm_get_mdinfoex(2, &value) == 0)
	{
	    if (value == -32767)
	    {
	        mutex_unlock(&TSMD_lock);
	        return -127000;
	    }
	    else
	    {
	        mutex_unlock(&TSMD_lock);
    	    return value*1000;
        }
	}
	mutex_unlock(&TSMD_lock);
	return -127000;
}
    
static int ntcpa_get_temp(struct thermal_zone_device *thermal,
             unsigned long *t)
{
	*t = ntcpa_get_hw_temp();

    // set new threshold
    {
        int i = ntcpa_num_trip - 1;
        for (; i >= 0; i--)
        {
            if (ntcpa_trip_temp[i] > *t)
            {
                mtk_mdm_set_mdinfoex_threshold(2, ntcpa_trip_temp[i]/1000);
                break;
            }
        }
    }
	
	if (*t > 85000)
        mtktspa_dprintk("%s temp=%lu\n", __func__, *t);
	return 0;
}

static int ntcpa_bind(struct thermal_zone_device *thermal,
                        struct thermal_cooling_device *cdev)
{
	int table_val=0;

	if(!strcmp(cdev->type, ntcpa_bind0))
	{
		table_val = 0;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind1))
	{
		table_val = 1;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind2))
	{
		table_val = 2;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind3))
	{
		table_val = 3;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind4))
	{
		table_val = 4;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind5))
	{
		table_val = 5;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind6))
	{
		table_val = 6;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind7))
	{
		table_val = 7;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind8))
	{
		table_val = 8;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind9))
	{
		table_val = 9;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else
		return 0;


	if (mtk_thermal_zone_bind_cooling_device(thermal, table_val, cdev)) 
	{
		mtktspa_dprintk("[%s] error binding cooling dev\n", __func__);
		return -EINVAL;
	} 
	else 
	{
		mtktspa_dprintk("[%s] binding OK\n", __func__);
	}

	return 0;
}

static int ntcpa_unbind(struct thermal_zone_device *thermal,
        struct thermal_cooling_device *cdev)
{
	int table_val=0;

	if(!strcmp(cdev->type, ntcpa_bind0))
	{
		table_val = 0;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind1))
	{
		table_val = 1;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind2))
	{
		table_val = 2;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind3))
	{
		table_val = 3;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind4))
	{
		table_val = 4;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind5))
	{
		table_val = 5;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind6))
	{
		table_val = 6;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind7))
	{
		table_val = 7;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind8))
	{
		table_val = 8;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else if(!strcmp(cdev->type, ntcpa_bind9))
	{
		table_val = 9;
		mtktspa_dprintk("[%s] %s\n", __func__, cdev->type);
	}
	else
		return 0;

	if (thermal_zone_unbind_cooling_device(thermal, table_val, cdev)) 
	{
		mtktspa_dprintk("[%s] error unbinding cooling dev\n", __func__);
		return -EINVAL;
	} 
	else 
	{
		mtktspa_dprintk("[%s] unbinding OK\n", __func__);
	}

	return 0;
}

static int ntcpa_get_mode(struct thermal_zone_device *thermal,
          enum thermal_device_mode *mode)
{
	*mode = (ntcpa_kernelmode) ? THERMAL_DEVICE_ENABLED
		: THERMAL_DEVICE_DISABLED;

	return 0;
}

static int ntcpa_set_mode(struct thermal_zone_device *thermal,
          enum thermal_device_mode mode)
{
	ntcpa_kernelmode = mode;
	return 0;
}

static int ntcpa_get_trip_type(struct thermal_zone_device *thermal, int trip,
         enum thermal_trip_type *type)
{
	*type = ntcpa_THERMAL_TRIP[trip];
	return 0;
}

static int ntcpa_get_trip_temp(struct thermal_zone_device *thermal, int trip,
         unsigned long *temp)
{
	*temp = ntcpa_trip_temp[trip];
	return 0;
}

static int ntcpa_get_crit_temp(struct thermal_zone_device *thermal,
         unsigned long *temperature)
{
	*temperature = 115000;
	return 0;
}

/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops ntcpa_dev_ops = {
	.bind = ntcpa_bind,
	.unbind = ntcpa_unbind,
	.get_temp = ntcpa_get_temp,
	.get_mode = ntcpa_get_mode,
	.set_mode = ntcpa_set_mode,
	.get_trip_type = ntcpa_get_trip_type,
	.get_trip_temp = ntcpa_get_trip_temp,
	.get_crit_temp = ntcpa_get_crit_temp,
};

static int ntcpa_read(struct seq_file *m, void *v)
{
	seq_printf(m, "[ntcpa_read] \ntrip_0_temp=%d \ntrip_1_temp=%d \ntrip_2_temp=%d \ntrip_3_temp=%d \ntrip_4_temp=%d \n\
trip_5_temp=%d \ntrip_6_temp=%d \ntrip_7_temp=%d \ntrip_8_temp=%d \ntrip_9_temp=%d \n\
g_THERMAL_TRIP_0=%d \ng_THERMAL_TRIP_1=%d \ng_THERMAL_TRIP_2=%d \ng_THERMAL_TRIP_3=%d \ng_THERMAL_TRIP_4=%d \n\
g_THERMAL_TRIP_5=%d \ng_THERMAL_TRIP_6=%d \ng_THERMAL_TRIP_7=%d \ng_THERMAL_TRIP_8=%d \ng_THERMAL_TRIP_9=%d \n\
cooldev0=%s \ncooldev1=%s \ncooldev2=%s \ncooldev3=%s \ncooldev4=%s \n\
cooldev5=%s \ncooldev6=%s \ncooldev7=%s \ncooldev8=%s \ncooldev9=%s \ntime_ms=%d\n",
				ntcpa_trip_temp[0],ntcpa_trip_temp[1],ntcpa_trip_temp[2],ntcpa_trip_temp[3],ntcpa_trip_temp[4],
				ntcpa_trip_temp[5],ntcpa_trip_temp[6],ntcpa_trip_temp[7],ntcpa_trip_temp[8],ntcpa_trip_temp[9],
				ntcpa_THERMAL_TRIP[0],ntcpa_THERMAL_TRIP[1],ntcpa_THERMAL_TRIP[2],ntcpa_THERMAL_TRIP[3],ntcpa_THERMAL_TRIP[4],
				ntcpa_THERMAL_TRIP[5],ntcpa_THERMAL_TRIP[6],ntcpa_THERMAL_TRIP[7],ntcpa_THERMAL_TRIP[8],ntcpa_THERMAL_TRIP[9],
				ntcpa_bind0,ntcpa_bind1,ntcpa_bind2,ntcpa_bind3,ntcpa_bind4,ntcpa_bind5,ntcpa_bind6,ntcpa_bind7,ntcpa_bind8,ntcpa_bind9,
				ntcpa_interval);
    
	return 0;
}

static ssize_t ntcpa_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	int len=0,time_msec=0;
	int trip[10]={0};
	int t_type[10]={0};
	int i;
	char bind0[20],bind1[20],bind2[20],bind3[20],bind4[20];
	char bind5[20],bind6[20],bind7[20],bind8[20],bind9[20];
	char desc[512];


	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';
		
	if (sscanf(desc, "%d %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d",
				&ntcpa_num_trip, &trip[0],&t_type[0],bind0, &trip[1],&t_type[1],bind1,
				&trip[2],&t_type[2],bind2, &trip[3],&t_type[3],bind3,
				&trip[4],&t_type[4],bind4, &trip[5],&t_type[5],bind5,
				&trip[6],&t_type[6],bind6, &trip[7],&t_type[7],bind7,
				&trip[8],&t_type[8],bind8, &trip[9],&t_type[9],bind9,
				&time_msec) == 32)
	{
		mtktspa_dprintk("[ntcpa_write] unregister_thermal\n");
		if (ntcpa_thz_dev) 
    	{
    		mtk_thermal_zone_device_unregister(ntcpa_thz_dev);
    		ntcpa_thz_dev = NULL;
    	}
	
		for(i=0; i<ntcpa_num_trip; i++)
			ntcpa_THERMAL_TRIP[i] = t_type[i];	

		ntcpa_bind0[0]=ntcpa_bind1[0]=ntcpa_bind2[0]=ntcpa_bind3[0]=ntcpa_bind4[0]=ntcpa_bind5[0]=ntcpa_bind6[0]=ntcpa_bind7[0]=ntcpa_bind8[0]=ntcpa_bind9[0]='\0';
				
		for(i=0; i<20; i++)
		{
			ntcpa_bind0[i]=bind0[i]; 
			ntcpa_bind1[i]=bind1[i]; 
			ntcpa_bind2[i]=bind2[i]; 
			ntcpa_bind3[i]=bind3[i]; 
			ntcpa_bind4[i]=bind4[i];
			ntcpa_bind5[i]=bind5[i]; 
			ntcpa_bind6[i]=bind6[i]; 
			ntcpa_bind7[i]=bind7[i]; 
			ntcpa_bind8[i]=bind8[i]; 
			ntcpa_bind9[i]=bind9[i];
		}

		mtktspa_dprintk("[ntcpa_write] g_THERMAL_TRIP_0=%d,g_THERMAL_TRIP_1=%d,g_THERMAL_TRIP_2=%d,g_THERMAL_TRIP_3=%d,g_THERMAL_TRIP_4=%d,\
g_THERMAL_TRIP_5=%d,g_THERMAL_TRIP_6=%d,g_THERMAL_TRIP_7=%d,g_THERMAL_TRIP_8=%d,g_THERMAL_TRIP_9=%d,\n",
				ntcpa_THERMAL_TRIP[0],ntcpa_THERMAL_TRIP[1],ntcpa_THERMAL_TRIP[2],ntcpa_THERMAL_TRIP[3],ntcpa_THERMAL_TRIP[4],
				ntcpa_THERMAL_TRIP[5],ntcpa_THERMAL_TRIP[6],ntcpa_THERMAL_TRIP[7],ntcpa_THERMAL_TRIP[8],ntcpa_THERMAL_TRIP[9]);
	mtktspa_dprintk("[ntcpa_write] cooldev0=%s,cooldev1=%s,cooldev2=%s,cooldev3=%s,cooldev4=%s,\
cooldev5=%s,cooldev6=%s,cooldev7=%s,cooldev8=%s,cooldev9=%s\n",
				ntcpa_bind0,ntcpa_bind1,ntcpa_bind2,ntcpa_bind3,ntcpa_bind4,ntcpa_bind5,ntcpa_bind6,ntcpa_bind7,ntcpa_bind8,ntcpa_bind9);

		for(i=0; i<ntcpa_num_trip; i++)
		{
			ntcpa_trip_temp[i]=trip[i];
		}

		ntcpa_interval=time_msec;

		mtktspa_dprintk("[ntcpa_write] trip_0_temp=%d,trip_1_temp=%d,trip_2_temp=%d,trip_3_temp=%d,trip_4_temp=%d,\
trip_5_temp=%d,trip_6_temp=%d,trip_7_temp=%d,trip_8_temp=%d,trip_9_temp=%d,time_ms=%d\n", 
				ntcpa_trip_temp[0],ntcpa_trip_temp[1],ntcpa_trip_temp[2],ntcpa_trip_temp[3],ntcpa_trip_temp[4],
				ntcpa_trip_temp[5],ntcpa_trip_temp[6],ntcpa_trip_temp[7],ntcpa_trip_temp[8],ntcpa_trip_temp[9],ntcpa_interval);

		mtktspa_dprintk("[ntcpa_write] register_thermal\n");
		ntcpa_thz_dev = mtk_thermal_zone_device_register("ntcpa", ntcpa_num_trip, NULL,
		    &ntcpa_dev_ops, 0, 0, 0, ntcpa_interval);

		return count;
	}
	else
	{
		mtktspa_dprintk("[ntcpa_write] bad argument\n");
	}
		
	return -EINVAL;
		
}

static int tzpantc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ntcpa_read, NULL);
}

static const struct file_operations tzpantc_fops = {
    .owner = THIS_MODULE,
    .open = tzpantc_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = ntcpa_write,
    .release = single_release,
};
#endif
#endif

void extra_thermal_zone_register(void)
{
#if defined(CONFIG_MTK_THERMAL_PA_VIA_ATCMD)
#if MTK_TS_PA_GET_MD_NTC_TEMP
    ntc6290_thz_dev = mtk_thermal_zone_device_register("ntcmd", ntc6290_num_trip, NULL,
		&ntc6290_dev_ops, 0, 0, 0, ntc6290_interval);
#endif

#if MTK_TS_PA_GET_MD_DRAM_TEMP
    dram6290_thz_dev = mtk_thermal_zone_device_register("drammd", dram6290_num_trip, NULL,
		&dram6290_dev_ops, 0, 0, 0, dram6290_interval);
#endif

#if MTK_TS_PA_GET_PA_NTC_TEMP
    ntcpa_thz_dev = mtk_thermal_zone_device_register("ntcpa", ntcpa_num_trip, NULL,
		&ntcpa_dev_ops, 0, 0, 0, ntcpa_interval);
#endif
#endif
}

void extra_proc_register(void)
{

#if defined(CONFIG_MTK_THERMAL_PA_VIA_ATCMD)
#if MTK_TS_PA_GET_MD_NTC_TEMP
		entry = proc_create("tzmdntc", S_IRUGO | S_IWUSR | S_IWGRP, mtktspa_dir, &tzmdntc_fops);
		if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
            proc_set_user(entry, 0, 1000);
#else
            entry->gid = 1000;
#endif
        }
#endif

#if MTK_TS_PA_GET_MD_DRAM_TEMP
        entry = proc_create("tzmddram", S_IRUGO | S_IWUSR | S_IWGRP, mtktspa_dir, &tzmddram_fops);
		if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
            proc_set_user(entry, 0, 1000);
#else
            entry->gid = 1000;
#endif
        }
#endif

#if MTK_TS_PA_GET_MD_NTC_TEMP
		entry = proc_create("tzpantc", S_IRUGO | S_IWUSR | S_IWGRP, mtktspa_dir, &tzpantc_fops);
		if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
            proc_set_user(entry, 0, 1000);
#else
            entry->gid = 1000;
#endif
        }
#endif

#endif
}

void extra_thermal_zone_unregister(void)
{
#if defined(CONFIG_MTK_THERMAL_PA_VIA_ATCMD)
#if MTK_TS_PA_GET_MD_NTC_TEMP
    if (ntc6290_thz_dev) 
	{
		mtk_thermal_zone_device_unregister(ntc6290_thz_dev);
		ntc6290_thz_dev = NULL;
	}
#endif
#if MTK_TS_PA_GET_MD_DRAM_TEMP
    if (dram6290_thz_dev) 
	{
		mtk_thermal_zone_device_unregister(dram6290_thz_dev);
		dram6290_thz_dev = NULL;
	}
#endif
#if MTK_TS_PA_GET_PA_NTC_TEMP
    if (ntcpa_thz_dev) 
	{
		mtk_thermal_zone_device_unregister(ntcpa_thz_dev);
		ntcpa_thz_dev = NULL;
	}
#endif
#endif
}
