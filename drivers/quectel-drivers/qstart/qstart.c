#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/mtd/mtd.h>
#include <linux/slab.h>
#include <linux/reboot.h>
#include <linux/kernel.h>
#include <linux/qstart.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>

#if 1 // def  QUECTEL_SYSTEM_BACKUP    // Ramos add for quectel for linuxfs restore

#define QUEC_IOCTL_SYS_REV_GET_INFO 		(0x00060001)
#define QUEC_IOCTL_SYS_REV_SET_INFO 		(0x00060002)

#define QUEC_BACKUP_MAGIC1        (0x78E5D4C2)
#define QUEC_BACKUP_MAGIC2        (0x54F7D60E)

#define CEFS_FILE_MAGIC1        (0x51D24368)
#define CEFS_FILE_MAGIC2        (0x4378AC6E)

//add by francis ,20180613,add data magic for linux
#define QUEC_DATA_MAGIC1        (0x5F3759DF)
#define QUEC_DATA_MAGIC2        (0x43A2E930)

#define  BACKUP_MTD_NUM 12
#define  PARTITION_NAME_LEN 64
#define QUEC_BACKUP_INFO_BLOCK_NUMS (3)  // keep with modem , bootloader aboot.c
//"sys_rev" partiton last QUEC_BACKUP_INFO_BLOCK_NUMS block use record restore information

#define QUEC_ALL_RESTORE_FLAG_BLOCK_INDEX (6) // the 3 block reserved for All parition restoring flag and fota upgraded flag
//the "sys_rev" partition last QUEC_ALL_RESTORE_FLAG_BLOCK_INDEX to QUEC_BACKUP_INFO_BLOCK_NUMS reserved for fota upgraded flag.

#define QUEC_START_TIMER_TOUT    30

#define BACKUP_INFO_BLOCK_NUMS (3)

static DEFINE_MUTEX(sys_rev_ioctl_lock);

typedef struct
{
  uint32_t magic1;  
  uint32_t magic2;
  uint32_t page_count;
  uint32_t data_crc;
  
  uint32_t reserve1;  
  uint32_t reserve2;
  uint32_t reserve3;
  uint32_t reserve4;
} quec_cefs_file_header_type;

typedef struct
{
    char ql_mtd_name[12];
    char ql_mtd_restore_name[12];
    uint64_t total_size;
    uint32_t ql_mtd_nub;
    uint32_t restore_flag;
    uint32_t restore_times;
    uint32_t backup_times;
    uint32_t crash[12];
} Ql_Mtd_Info;
// add by [francis.huan],20180417,for partition info who need to restore or erase

typedef struct
{
  uint32_t magic1;  
  uint32_t magic2;

//add by [francis],20180524,match for modem ,make sure cefs is ok
  uint32_t cefs_restore_flag;
  uint32_t cefs_restore_times;
  uint32_t cefs_backup_times;
  uint32_t cefs_crash[10];  

    uint32_t linuxfs_restore_flag;
    uint32_t linuxfs_restore_times;
    uint32_t linuxfs_backup_times;
    uint32_t linuxfs_crash[10]; 

    // modem backup restore flag
    uint32_t modem_restore_flag;
    uint32_t modem_restore_times;
    uint32_t modem_backup_times;
    uint32_t modem_crash[10];  

    //  recovery restore flag

    // other image restore flag
    uint32_t image_restoring_flag;
    uint32_t reserved1;
    uint32_t reserved2[100];
 
    uint32_t data_magic1;
    uint32_t data_magic2;

  Ql_Mtd_Info ql_mtd_info[BACKUP_MTD_NUM];
} quec_backup_info_type;

typedef struct
{
	char * partition_name[PARTITION_NAME_LEN];
	int crash_where;
} quec_restore_info_type;

static    quec_backup_info_type Flag_msg_init={0,0,0,0,0,{0},0,0,0,{0},0,0,0,{0},0,0,{0},0,0,
{
#if   1  //add data partition  for restore
    {"modem","b_modem",0,0,0,0,0,0},
    {"b_modem","modem",0,0,0,0,0,0},
    {"system","b_system",0,0,0,0,0,0},
    {"b_system","system",0,0,0,0,0,0},
    {"oemapp","b_oemapp",0,0,0,0,0,0},
    {"b_oemapp","oemapp",0,0,0,0,0,0},
    {"userdata","",0,0,0,0,0,0},
    {"persist","",0,0,0,0,0,0},
    {"quecrw","",0,0,0,0,0,0},
    {"boot", "b_boot", 0,0,0,0,0,0},	
    {"b_boot", "boot", 0,0,0,0,0,0},	
#endif

    }
};




typedef struct
{
    uint32_t All_Restoring_flag;
    uint32_t fota_upgradedFlag; // module have upgraded fota flag 
    uint32_t reserved2;
} quec_All_RestoringInfo;


struct qstart_device_t{
    struct miscdevice misc;
};

static struct mtd_info *mtd = NULL;
static loff_t qfirst_goodblock_addr = 0;

struct qstart_device_t *qstart_devp;
struct timer_list qstart_poll_timer;
struct work_struct qstart_timer_work;

static DEFINE_MUTEX(qstart_timer_lock);

static int start_mode_set(const char *val, struct kernel_param *kp);
static int start_mode = 0;
module_param_call(start_mode, start_mode_set, param_get_int, &start_mode, 0644);

//add by len, get the recovery value from cmdline, 2018-1-18
bool boot_mode = false;
//static unsigned int bootmode = 1;
//
///**
// * Author : Darren
// * Data : 2017/6/30
// * parse cmdline to get current boot mode
// * the function is compatible when we set 
// * SYSTEM_MTD_NUMBER right.
// */
//static int __init cmdline_parse_bootmode(char *str)
//{
//    unsigned int val;
//    kstrtouint(str,10,&val);
//    if (val == SYSTEM_MTD_NUMBER)
//        bootmode = SYSTEM_MTD_NUMBER;
//    else if (val == RECOVERYFS_MTD_NUMBER)
//        bootmode = RECOVERYFS_MTD_NUMBER;
//    return 0;
//}
//early_param("ubi.mtd", cmdline_parse_bootmode);
static int __init quectel_set_bootmode(char *str)
{
    if (str) {
        boot_mode = true;
    }
    else {
        boot_mode = false;
    }
    return 0;
}
early_param("recovery", quectel_set_bootmode);
//add end

/**
 * Author : Darren
 * Date : 2017/6/30
 * get_bootmode - will return current start mode
 * 0 -- normode, system mount linuxfs as rootfs
 * 1 -- recoverymode, system mount recoveryfs as rootfs
 */
bool get_bootmode(unsigned int *mode)
{
    return boot_mode;
}
EXPORT_SYMBOL(get_bootmode);

static int start_mode_set(const char *val, struct kernel_param *kp)
{
    int ret;

    ret = param_set_int(val,kp);

    if(ret)
        return ret;
    
    return 0;
}

static void qstart_poll_timer_cb(void)
{
    static unsigned int count = 0;

    mutex_lock(&qstart_timer_lock);
    count++;
    if(count >=  QUEC_START_TIMER_TOUT && 0 == start_mode)
    {
        printk("\n\n\nStart mode = %d, count=%d\n\n\n", start_mode,count);
        schedule_work(&qstart_timer_work);    
           //panic("Quec Start Timer error!");     
    }
    mutex_unlock(&qstart_timer_lock);

    if(1 == start_mode)
        del_timer(&qstart_poll_timer);
    else
         mod_timer(&qstart_poll_timer,jiffies + HZ);
}

static struct qstart_device_t qstart_device = {
    .misc = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "qstart",
    }
};

/******************************************************************************************
who-2018/06/25:Description....
Refer to [Issue-Depot].[IS0000197][Submitter:ramos.zhang,Date:2018-06-25]
<ģ���ֳ�efs�����޷���ԭmodem��ͣ��������cfun=7�޷�����>
******************************************************************************************/
unsigned int Quectel_Is_EFS_Backup_Valid(void)
{
    size_t readlen = 0;
    quec_cefs_file_header_type BackupCefs_Info;
    uint32_t crc = 0;
    uint32_t i =0;
    uint64_t mtd_size;
    unsigned char *onepage = NULL;
    struct mtd_info *mtd = NULL;

    mtd = get_mtd_device_nm("sys_rev");
    if(IS_ERR(mtd))
    {
        printk("@Ramos get sys_rev mtd fail.!\r\n");
        goto efsBackupInvalid;
    }
    else
    {
        mtd_size = mtd->size;
        printk("@Ramos mtd->writesize =%d, mtd->erasesize:%d  blockcount\n",  mtd->writesize,mtd->erasesize);
        for(i=0; qfirst_goodblock_addr < mtd_size; i++)
        {
            qfirst_goodblock_addr = i * mtd->erasesize; 
            if(!mtd_block_isbad(mtd,qfirst_goodblock_addr)) 
                break;
        }

        onepage = kmalloc(mtd->writesize, GFP_KERNEL);
        if(NULL == onepage)
        {
            printk("@Ramos memory is not enough to onepage, line=%d\n", __LINE__);
            goto  efsBackupInvalid;
        }
        
        memset(onepage, 0x00, mtd->writesize);
        mtd_read(mtd, qfirst_goodblock_addr, mtd->writesize, &readlen ,onepage);
        if(readlen != mtd->writesize )
        {
            printk("@Ramos read Flag Failed!!,line=%d\r\n\r\n",__LINE__);
            goto  efsBackupInvalid;
        }
        memset((void *)&BackupCefs_Info, 0x00, sizeof(quec_cefs_file_header_type));
        memcpy((void *)&BackupCefs_Info, onepage, sizeof(quec_cefs_file_header_type));

        if((CEFS_FILE_MAGIC1 != BackupCefs_Info.magic1) || (CEFS_FILE_MAGIC2 != BackupCefs_Info.magic2))
        {
            printk("@Ramos efs2 restore file magic1 error !!!\r\n\r\n");
            goto  efsBackupInvalid;
        }
    }

    return 1;
efsBackupInvalid:
    if(onepage != NULL)
    {
        kfree(onepage);
        onepage = NULL;
    }
    return 0;
}


unsigned int Quectel_Is_Partition_Exist(const char * sourc_partition)
{
    struct mtd_info *mtd = NULL;

    //printk("@Quectel0125 check [%s]partition is exist ? \r\n",sourc_partition);
    mtd = get_mtd_device_nm(sourc_partition);
    if(IS_ERR(mtd))
    {
        printk("@Quectel0125 can't get [%s]partition !!!\r\n",sourc_partition);
        return 0;
    }
    else
    {
        printk("@Quectel0125 get partition[%s] mtd_nm=%d\r\n",sourc_partition, mtd->index);
        return 1;
    }
}

unsigned int Quectel_Is_BackupPartition_Exist(const char * sourc_partition)
{
    struct mtd_info *mtd = NULL;

    //printk("@Quectel0125 check backup [%s]partition is exist ? \r\n",sourc_partition);
    mtd = get_mtd_device_nm(sourc_partition);
    if(IS_ERR(mtd))
    {
        printk("@Quectel0125 can't get backup  [%s]partition !!!\r\n",sourc_partition);
        return 0;
    }
    else
    {
        if(!strcmp(sourc_partition,"efs2"))
        {
            if(Quectel_Is_EFS_Backup_Valid())
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
        return 1;
    }
}

// modify by [francis.huan],20180417,priortity partition_name > mtd_nub,then setting RestoreFlag
unsigned int Quectel_Set_Partition_RestoreFlag(const char * partition_name,int mtd_nub, int where)
{
    unsigned char *onepage = NULL;
    quec_backup_info_type Flag_msg;
    struct erase_info ei;
    size_t write = 0;
    size_t readlen = 0;
    int err,i,update_flag=0;
    int ret = 0;
    uint64_t tmp;
    uint32_t blocksize=0;
    if(where>11)
    {
	    printk("ERROR !!!!!!! @Quectel0125 the value of where is bigger than 11, will change the value of where to 0\r\n");
	    where = 0;
    }
    //set system restore flag in sys_rev partition last 3 block
    mtd = get_mtd_device_nm("sys_rev");
    if(IS_ERR(mtd))
    {
        printk("@Quectel0125 get sys_rev mtd fail.!\r\n");
        return 0;
    }
    else
    {
        tmp = mtd->size; // totle size .
        blocksize =  mtd->erasesize;
        
        printk("@Quectel0125 :mtdsize:0x%llx, mtd->writesize =%d, mtd->erasesize:%d  \n", tmp, mtd->writesize,mtd->erasesize);
        for(i=0; i<QUEC_BACKUP_INFO_BLOCK_NUMS; i++)
        {
            qfirst_goodblock_addr =tmp -(QUEC_BACKUP_INFO_BLOCK_NUMS- i) * mtd->erasesize; 
            if(!mtd_block_isbad(mtd,qfirst_goodblock_addr)) 
                break;
        }

        onepage = kmalloc(mtd->writesize, GFP_KERNEL);
        if(NULL == onepage)
        {
            printk("@Quectel0125 memory is not enough to onepage\n");
            return ret;
        }
        
        printk("@Quectel0125 qfirst_goodblock_addr=0x%llx \n", qfirst_goodblock_addr);
        memset(onepage, 0x00, mtd->writesize);
        mtd_read(mtd, qfirst_goodblock_addr, mtd->writesize, &readlen ,onepage);
        if(readlen != mtd->writesize )
        {
            printk("@Quectel0125 read Flag Failed!!\r\n\r\n");
        }


        memcpy((void *)&Flag_msg,(void *)onepage,sizeof(Flag_msg));
        printk("@Quectel0125 set partition(%s) restore,print magic1=0x%x,magic2=0x%x,flag_size[%d]\r\n", partition_name,Flag_msg.magic1,Flag_msg.magic2,sizeof(Flag_msg));
        if(QUEC_BACKUP_MAGIC1 != Flag_msg.magic1 || QUEC_BACKUP_MAGIC2 != Flag_msg.magic2)
        {
            //first time write to 
            memset((void *)&Flag_msg,0x00,sizeof(Flag_msg));
        }
        
        if( 0xAA55 == Flag_msg.linuxfs_backup_times)
        {
            printk("@Quectel0125 AT configed    not allow  linux fs restore  linuxfs_backup_times=%d\r\n\r\n\r\n", Flag_msg.linuxfs_backup_times);
            return 0;
        }

        int i;            
        for ( i = 0 ;i < BACKUP_MTD_NUM;i++)
        {
            if(Flag_msg_init.ql_mtd_info[i].ql_mtd_name[0] != 0)
            {
                if(!IS_ERR(get_mtd_device_nm(Flag_msg_init.ql_mtd_info[i].ql_mtd_name))){
                    Flag_msg_init.ql_mtd_info[i].ql_mtd_nub=get_mtd_device_nm(Flag_msg_init.ql_mtd_info[i].ql_mtd_name)->index;
                    Flag_msg_init.ql_mtd_info[i].total_size=get_mtd_device_nm(Flag_msg_init.ql_mtd_info[i].ql_mtd_name)->size;
                }
                // get mtd nub for src partition
                //if(strcmp(Flag_msg.ql_mtd_info[i].ql_mtd_name,Flag_msg_init.ql_mtd_info[i].ql_mtd_name))
                if(true) //always get mtd info , will error if partition update 
                {
                    strcpy(Flag_msg.ql_mtd_info[i].ql_mtd_name,Flag_msg_init.ql_mtd_info[i].ql_mtd_name);
                    strcpy(Flag_msg.ql_mtd_info[i].ql_mtd_restore_name,Flag_msg_init.ql_mtd_info[i].ql_mtd_restore_name);
                    Flag_msg.ql_mtd_info[i].ql_mtd_nub =  Flag_msg_init.ql_mtd_info[i].ql_mtd_nub;
                    Flag_msg.ql_mtd_info[i].total_size =  Flag_msg_init.ql_mtd_info[i].total_size;                    
                }
            }else {    
                strcpy(Flag_msg.ql_mtd_info[i].ql_mtd_name,"");
            }
        }
        Flag_msg.magic1 =  QUEC_BACKUP_MAGIC1;
        Flag_msg.magic2 =  QUEC_BACKUP_MAGIC2;
              Flag_msg.data_magic1 =  QUEC_DATA_MAGIC1;
              Flag_msg.data_magic2 =  QUEC_DATA_MAGIC2;    
        if(!strcmp(partition_name,"efs2"))
        {
            Flag_msg.cefs_restore_flag = 0x10;
            update_flag = 1;//Ramos.zhang-20190114 resolve efs partition not restore when  modem fatal error 4 times
        }

        printk("@Quectel0125 check need restore partition mtd_nub=%d\n",mtd_nub );
        for ( i = 0 ;i < BACKUP_MTD_NUM;i++)
        {
            if(Flag_msg.ql_mtd_info[i].ql_mtd_name[0] != 0)
            {
                //printk("@Quectel0125 mtd_name[%s]\n",Flag_msg.ql_mtd_info[i].ql_mtd_name );
                if( partition_name[0] == 0 )
                {
                    if(Flag_msg.ql_mtd_info[i].ql_mtd_nub == mtd_nub)
                    {
                        if(!Quectel_Is_Partition_Exist(Flag_msg.ql_mtd_info[i].ql_mtd_name))
                        {
                            printk("111 the [%s] parition no exist !!!\n",Flag_msg.ql_mtd_info[i].ql_mtd_name);
                            return 0;
                        }
                        if(0 /*!Quectel_Is_BackupPartition_Exist(Flag_msg.ql_mtd_info[i].ql_mtd_restore_name)*/)// not need, cause not backupPartiiton (eg:usr_data)  catn't erase when system carshed
                        {
                            printk("111 the backup  [%s] parition no exist !!!\n",Flag_msg.ql_mtd_info[i].ql_mtd_restore_name);
                            return 0;
                        }
                        Flag_msg.ql_mtd_info[i].restore_flag = 1;
                        Flag_msg.ql_mtd_info[i].crash[where] +=1;
                        update_flag = 1;
                        printk("@Qeuctel_0125  mtd=%d  set Restore in where=%d\n", mtd_nub, where);
                    }
                }else
                {
                    if(!strcmp(Flag_msg.ql_mtd_info[i].ql_mtd_name,partition_name))
                    {
                        if(!Quectel_Is_Partition_Exist(Flag_msg.ql_mtd_info[i].ql_mtd_name))
                        {
                            printk("222 the [%s] parition no exist !!!\n",Flag_msg.ql_mtd_info[i].ql_mtd_name);
                            return 0;
                        }    
                        if(0/*!Quectel_Is_BackupPartition_Exist(Flag_msg.ql_mtd_info[i].ql_mtd_restore_name)*/)// not need, cause not backupPartiiton (eg:usr_data)  catn't erase when system carshed
                        {
                            printk("222 the backup  [%s] parition no exist !!!\n",Flag_msg.ql_mtd_info[i].ql_mtd_restore_name);
                            return 0;
                        }
                        Flag_msg.ql_mtd_info[i].restore_flag = 1;
                        Flag_msg.ql_mtd_info[i].crash[where] +=1;
                        update_flag = 1;
                        printk("@Qeuctel_0125  partition[%s] set Restore in where=%d\n", partition_name, where);
                    }
                }
            }    

        }
        
    if(1 == update_flag)
    {
        memset(&ei, 0, sizeof(struct erase_info));
        ei.mtd = mtd;
        ei.addr = qfirst_goodblock_addr;
        ei.len = mtd->erasesize;
        err = mtd_erase(mtd, &ei); //ȲһҪд
        
        memcpy((void *)onepage,(void *)&Flag_msg, sizeof(Flag_msg));
        err = mtd_write(mtd, qfirst_goodblock_addr, mtd->writesize, &write, onepage);
        if(err || write != mtd->writesize)
        {
            printk("@Quectel0125 set partition(%s) Flag  failed at %#llx\n", partition_name,(long long)qfirst_goodblock_addr);
            goto exit;
        }
        ret = 1;
        machine_restart(NULL);
    }
/*
        memset(onepage, 0x00, mtd->writesize);
        mtd_read(mtd, qfirst_goodblock_addr, mtd->writesize, &readlen ,onepage);
        printk("@Quectel0125 debug Restore flag is:%s\n",  ((struct restorflag_message *)onepage)->command);
*/
        
    }    

exit:
    if(onepage != NULL)
    {
        kfree(onepage);
        onepage = NULL;
    }
    return ret;
}

void Quectel_Partition_Restore(const char * partition_name,int mtd_nub, int where)
{

    Quectel_Set_Partition_RestoreFlag(partition_name, mtd_nub ,where);
    return 0;

}

static int Quectel_Read_Backup_Info(quec_backup_info_type * backup_info)
{
	unsigned char *onepage = NULL;
	int retval = 0;
	int i = 0;
	int readlen = 0;
	unsigned long tmp;
	if(NULL == backup_info) {
		retval = -1;
		goto end;
	}
	
	mtd = get_mtd_device_nm("sys_rev");
    	if(IS_ERR(mtd))
    	{
        	printk("@Quectel0125 get sys_rev mtd fail.!\r\n");
        	retval = -1;
		goto end;
    	}
	
    	tmp = mtd->size;    
  
    	for(i=0; i<QUEC_BACKUP_INFO_BLOCK_NUMS; i++) {
        	qfirst_goodblock_addr =tmp -(QUEC_BACKUP_INFO_BLOCK_NUMS- i) * mtd->erasesize; 
        	if(!mtd_block_isbad(mtd,qfirst_goodblock_addr)) 
			break;
   	}
	
	onepage = kmalloc(mtd->writesize, GFP_KERNEL);
    	if(NULL == onepage) {
        	printk("@Quectel0125 memory is not enough to onepage\n");
        	retval = -1;
		goto end;
    	}
	
    	printk("@Quectel0125 qfirst_goodblock_addr=0x%llx \n", qfirst_goodblock_addr);
    	memset(onepage, 0x00, mtd->writesize);
    	mtd_read(mtd, qfirst_goodblock_addr, mtd->writesize, &readlen ,onepage);
    	if(readlen != mtd->writesize ) {
        	printk("@Quectel0125 read Flag Failed!!\r\n\r\n");
		retval = -1;
		goto end;
    	}
	
	memcpy((void *)backup_info,(void *)onepage,sizeof(quec_backup_info_type));
	
	end: 
		if(onepage != NULL) {
			kfree(onepage);
			onepage = NULL;
		}
		return 0;
	
}



void Quectel_Erase_Partition(const char * partition_name)
{
    struct mtd_info *mtd = NULL;
    struct erase_info ei;
    int err,i;
    
    printk("@Quectel0125 there are fatal errror on the %s partition , we must erase it !!!\r\n", partition_name);
    mtd = get_mtd_device_nm(partition_name);
    if(IS_ERR(mtd))
    {
        printk("ERROR!!!!!  @Quectel0125 get  %s mtd fail.!\r\n", partition_name);
        return 0;
    }
    else
    {
        memset(&ei, 0, sizeof(struct erase_info));
        ei.mtd = mtd;
        ei.len = mtd->erasesize;
        for(i=0 ;  ; i++)
        {
            ei.addr = i*(mtd->erasesize);
            if(ei.addr  > mtd->size)
            {
                break;
            }
            err = mtd_erase(mtd, &ei); //
        }
    }
        
    machine_restart(NULL);

return 0;
}

EXPORT_SYMBOL(Quectel_Set_Partition_RestoreFlag);
EXPORT_SYMBOL(Quectel_Partition_Restore);
EXPORT_SYMBOL(Quectel_Erase_Partition);

static void qstart_timer_handle(struct work_struct *work)
{

    Quectel_Set_Partition_RestoreFlag("system",-1,10);
}

static void __exit qstart_exit(void)
{
    misc_deregister(&qstart_device.misc);
}

static int __init qstart_init(void)
{
    int ret;

    printk("@Quectel0125 Qstart_init entry !!!\r\n");

#if 0   //QUECTEL_LINUX_APP_DETECT
    qstart_devp = &qstart_device;
    init_timer(&qstart_poll_timer);
                qstart_poll_timer.function = (void *)qstart_poll_timer_cb; 
                qstart_poll_timer.expires = jiffies + HZ;
                add_timer(&qstart_poll_timer);
    
    INIT_WORK(&qstart_timer_work, qstart_timer_handle);        
    
    ret = misc_register(&qstart_device.misc);
#endif

    return ret;
}

module_init(qstart_init);
module_exit(qstart_exit);

MODULE_DESCRIPTION("QUECTEL Start Driver");
MODULE_LICENSE("GPL v2");

static int sys_rev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int sys_rev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long sys_rev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int error = 0;
	bool ret = 0;
	int iter = 0;
	quec_backup_info_type backup_info ;
	quec_restore_info_type restore_info;
	mutex_lock(&sys_rev_ioctl_lock);
	
	memset(&backup_info, 0x00, sizeof(quec_backup_info_type));
	memset(&restore_info, 0x00, sizeof(quec_restore_info_type));

	switch (cmd) {
	case QUEC_IOCTL_SYS_REV_GET_INFO:	
		ret = Quectel_Read_Backup_Info((quec_backup_info_type*)&backup_info);
		if (ret == 0) {
			if (copy_to_user((void __user *)arg, &backup_info, sizeof(quec_backup_info_type))) {
				error = -1;
			}
		} else {
			error = -1;
		}

		break;

	case QUEC_IOCTL_SYS_REV_SET_INFO:
		if (copy_from_user(&restore_info, (quec_restore_info_type __user *) arg,
				 sizeof(quec_restore_info_type))) {
			error = -1;
			break;
		}
		if(!strcmp(restore_info.partition_name,"efs2")) {
			Quectel_Partition_Restore(restore_info.partition_name,-1,restore_info.crash_where);
			error = 0;
			break;
		}
		
		while(iter<BACKUP_MTD_NUM)
		{
			if(!strcmp(Flag_msg_init.ql_mtd_info[iter].ql_mtd_name,restore_info.partition_name)){
				Quectel_Partition_Restore(restore_info.partition_name,-1,restore_info.crash_where);
				error = 0;
				break;
			}
			iter ++;
		}

		error = -1;
		break;
	default:
		error = -1;
	}

out:
	mutex_unlock(&sys_rev_ioctl_lock);
	return error;
}

static const struct file_operations sys_rev_fops = {
	.owner = THIS_MODULE,
	.open = sys_rev_open,
	.unlocked_ioctl = sys_rev_ioctl,
	.llseek = NULL,
	.release = sys_rev_release,
};

static int __init quec_sys_rev_proc_init(void)
{
	proc_create("quec_sys_rev", 0, NULL, &sys_rev_fops);
	return 0;
}

__initcall(quec_sys_rev_proc_init);

#endif
