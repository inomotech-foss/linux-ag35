#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/stringify.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/uaccess.h>

#define QFPROM_PHY_BASE					0xa4000
#define QFPROM_ROM_SIZE					0x1000

#define QFPROM_BYTES_PER_FUSE_ROW      	8
#define QFPROM_JTAG_ID_ROW_ADDR         (36*QFPROM_BYTES_PER_FUSE_ROW)
#define QFPROM_SERIAL_NUM_ROW_ADDR      (37*QFPROM_BYTES_PER_FUSE_ROW)
#define QFPROM_OEM_ID_ROW_ADDR          (43*QFPROM_BYTES_PER_FUSE_ROW)
#define QFPROM_OEM_SEC_BOOT_ROW_ADDR    (58*QFPROM_BYTES_PER_FUSE_ROW)
#define QFPROM_PK_HASH0_ROW_ADDR		(60*QFPROM_BYTES_PER_FUSE_ROW)
#define QFPROM_PK_HASH1_ROW_ADDR		(61*QFPROM_BYTES_PER_FUSE_ROW)
#define QFPROM_PK_HASH2_ROW_ADDR		(62*QFPROM_BYTES_PER_FUSE_ROW)
#define QFPROM_PK_HASH3_ROW_ADDR		(63*QFPROM_BYTES_PER_FUSE_ROW)
#define QFPROM_PK_HASH4_ROW_ADDR		(64*QFPROM_BYTES_PER_FUSE_ROW)

#define OEM_SECURE_BOOT1_AUTH_EN    	(1 << 5)
#define OEM_SECURE_BOOT2_AUTH_EN    	(1 << 13)
#define OEM_SECURE_BOOT3_AUTH_EN    	(1 << 21)
#define OEM_SECURE_BOOT1_SN_EN          (1 << 6)

void __iomem *qfprom_base = NULL;

static ssize_t secboot_state_show(struct class *class, struct class_attribute *attr, char *buf)
{
    uint64_t fuse_val;
    
    fuse_val = readq_relaxed(qfprom_base + QFPROM_OEM_SEC_BOOT_ROW_ADDR);
    
    if (fuse_val & OEM_SECURE_BOOT1_AUTH_EN) {
        sprintf(buf, "%s\n","auth_enable");
    } else {
        sprintf(buf, "%s\n", "auth_disable");
    }

    return strlen(buf) + 1;
}

static ssize_t secboot_rootca_hash_show(struct class *class, struct class_attribute *attr, char *buf)
{
    uint64_t fuse_val;
    uint8_t hash[32];
    uint32_t i;
    
    for (i = 0; i < 4; i++) {
        fuse_val = readq_relaxed(qfprom_base + QFPROM_PK_HASH0_ROW_ADDR + i*QFPROM_BYTES_PER_FUSE_ROW);
        memcpy(&hash[i * 7], &fuse_val, 7);
    }
    fuse_val = readq_relaxed(qfprom_base + QFPROM_PK_HASH4_ROW_ADDR);
    memcpy(&hash[28], &fuse_val, 4);
    
    for(i = 0; i < 32; i++) {
         sprintf(&buf[2 * i],"%02x",hash[i]);
    }
    buf[64] = '\n';
    buf[65] = '\0';
    
    return strlen(buf) + 1;
}

static ssize_t secboot_SN_state_show(struct class *class, struct class_attribute *attr, char *buf)
{
    uint64_t fuse_val;

    fuse_val = readq_relaxed(qfprom_base + QFPROM_OEM_SEC_BOOT_ROW_ADDR);
    if (fuse_val & OEM_SECURE_BOOT1_SN_EN) {
        sprintf(buf, "%s\n", "sn_enable");
    } else {
        sprintf(buf, "%s\n", "sn_disbale");
    }

    return strlen(buf)+1;
}

static ssize_t MSM_HW_ID_show(struct class *class, struct class_attribute *attr, char *buf)
{
    uint64_t fuse_val;
    uint32_t serial;
    uint32_t jtag_id;
    uint32_t jtag;
    uint32_t model_id;
    uint32_t oem_id;
    uint8_t msm[8] = {0};

    fuse_val = readq_relaxed(qfprom_base + QFPROM_OEM_SEC_BOOT_ROW_ADDR);
    if (fuse_val & OEM_SECURE_BOOT1_SN_EN) {
        fuse_val = readq_relaxed(qfprom_base + QFPROM_SERIAL_NUM_ROW_ADDR);
        serial = (uint32_t)(fuse_val & 0xffffffff);
    
        msm[7] = (uint8_t)(serial & 0xff);
        msm[6] = (uint8_t)((serial >> 8) & 0xff);
        msm[5] = (uint8_t)((serial >> 16) & 0xff);
        msm[4] = (uint8_t)((serial >> 24) & 0xff);
    } else {
        fuse_val = readq_relaxed(qfprom_base + QFPROM_OEM_ID_ROW_ADDR);
        oem_id = (uint32_t)(fuse_val & 0xffff);
        model_id = (uint32_t)((fuse_val >> 16) & 0xffff);

        msm[7] = (uint8_t)(model_id & 0xff);
        msm[6] = (uint8_t)((model_id >> 8) & 0xff);
        msm[5] = (uint8_t)(oem_id & 0xff);
        msm[4] = (uint8_t)((oem_id >> 8) & 0xff);
    }

    fuse_val = readq_relaxed(qfprom_base + QFPROM_JTAG_ID_ROW_ADDR);
    jtag_id = (uint32_t)(fuse_val & 0xfffff);
    jtag  = ((jtag_id << 12) | 0xe1) & 0xfffffff;

    msm[3] = (uint8_t)(jtag & 0xff);
    msm[2] = (uint8_t)((jtag >> 8) & 0xff);
    msm[1] = (uint8_t)((jtag >> 16) & 0xff);
    msm[0] = (uint8_t)((jtag >> 24) & 0xff);

    sprintf(buf, "%02x%02x%02x%02x%02x%02x%02x%02x\n", msm[0], msm[1], msm[2], msm[3], msm[4], msm[5], msm[6], msm[7]);

    return strlen(buf)+1;
}

static struct class_attribute qtee_class_attrs[] = {
	__ATTR(secboot_state, S_IRUGO, secboot_state_show, NULL),
	__ATTR(secboot_rootca_hash, S_IRUGO, secboot_rootca_hash_show, NULL),
    __ATTR(secboot_SN_state, S_IRUGO, secboot_SN_state_show, NULL),
    __ATTR(MSM_HW_ID, S_IRUGO, MSM_HW_ID_show, NULL),
	__ATTR_NULL
};

static struct class sec_class = {
	.owner = THIS_MODULE,
	.name = "qtee",
	.class_attrs = qtee_class_attrs,
};


static int __init qtee_init(void)
{
    int ret;

	qfprom_base = ioremap(QFPROM_PHY_BASE, QFPROM_ROM_SIZE);
	if (qfprom_base == NULL)
		return -ENOMEM;

    ret = class_register(&sec_class);
    if (ret)
        goto error;

    return 0;

error:
    return ret;
}

static void __exit qtee_exit(void)
{
	class_unregister(&sec_class);
	iounmap(qfprom_base);
}

module_init(qtee_init);
module_exit(qtee_exit);
MODULE_LICENSE("GPL v2");

