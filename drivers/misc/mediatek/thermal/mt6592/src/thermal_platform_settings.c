
//************************************
// Extern functions and variables
//************************************
extern unsigned int mt_socfreq_get_cur_gpufreq(void);

unsigned int get_cur_gpu_freq_wapper(void)
{
	//Return MHz
	return mt_socfreq_get_cur_gpufreq()/1000;
}
