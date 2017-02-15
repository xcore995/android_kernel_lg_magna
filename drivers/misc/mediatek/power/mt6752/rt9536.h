#ifndef __RT9536_CHARGER_H__
#define __RT9536_CHARGER_H__

struct rt9536_platform_data {
	/* gpio */
	int en_set;
	int chgsb;

	/* features */
	int hv_enable;
};

#endif /* __RT9536_CHARGER_H__ */
