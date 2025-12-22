#include <linux/capability.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/smp.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <asm/apic.h>

#define SGXVEC_DISABLE_APIC_CMD 0
#define SGXVEC_ENABLE_APIC_CMD 1
#define SGXVEC_RESET_ALL_CMD 2

struct sgxvec_cmd {
  int mode;
  int cpuid;
};

static DEFINE_MUTEX(g_sgxvec_lock);
static DECLARE_BITMAP(g_apic_disabled_mask, NR_CPUS);

static void sgxvec_disable_apic(void *info) {
  unsigned int v;
  if (apic_is_x2apic_enabled()) {
#ifdef CONFIG_X86_X2APIC
    v = native_apic_msr_read(APIC_SPIV);
#else
    return;
#endif
  } else {
    v = native_apic_mem_read(APIC_SPIV);
  }
  v &= ~(APIC_SPIV_APIC_ENABLED);
  if (apic_is_x2apic_enabled()) {
#ifdef CONFIG_X86_X2APIC
    native_apic_msr_write(APIC_SPIV, v);
#endif
  } else {
    native_apic_mem_write(APIC_SPIV, v);
  }
}

static void sgxvec_enable_apic(void *info) {
  unsigned int v;
  if (apic_is_x2apic_enabled()) {
#ifdef CONFIG_X86_X2APIC
    v = native_apic_msr_read(APIC_SPIV);
#else
    return;
#endif
  } else {
    v = native_apic_mem_read(APIC_SPIV);
  }
  v |= (APIC_SPIV_APIC_ENABLED);
  if (apic_is_x2apic_enabled()) {
#ifdef CONFIG_X86_X2APIC
    native_apic_msr_write(APIC_SPIV, v);
#endif
  } else {
    native_apic_mem_write(APIC_SPIV, v);
  }
}

static void sgxvec_enable_apic_all(void) {
  int cpu;
  for_each_online_cpu(cpu) {
    if (test_bit(cpu, g_apic_disabled_mask)) {
      (void)smp_call_function_single(cpu, sgxvec_enable_apic, NULL, 1);
      clear_bit(cpu, g_apic_disabled_mask);
    }
  }
}

static ssize_t sgxvec_proc_write(struct file *file, const char __user *buffer,
                                 size_t count, loff_t *ppos) {
  struct sgxvec_cmd cmd;
  int rc;

  (void)file;
  (void)ppos;

  if (!capable(CAP_SYS_ADMIN)) {
    return -EPERM;
  }
  if (count != sizeof(cmd)) {
    return -EINVAL;
  }
  if (copy_from_user(&cmd, buffer, sizeof(cmd))) {
    return -EFAULT;
  }

  mutex_lock(&g_sgxvec_lock);
  switch (cmd.mode) {
  case SGXVEC_DISABLE_APIC_CMD:
    if (cmd.cpuid < 0 || cmd.cpuid >= nr_cpu_ids) {
      rc = -EINVAL;
      break;
    }
    if (!cpu_online(cmd.cpuid)) {
      rc = -EINVAL;
      break;
    }
    rc = smp_call_function_single(cmd.cpuid, sgxvec_disable_apic, NULL, 1);
    if (!rc) {
      set_bit(cmd.cpuid, g_apic_disabled_mask);
    }
    break;
  case SGXVEC_ENABLE_APIC_CMD:
    if (cmd.cpuid < 0 || cmd.cpuid >= nr_cpu_ids) {
      rc = -EINVAL;
      break;
    }
    if (!cpu_online(cmd.cpuid)) {
      rc = -EINVAL;
      break;
    }
    rc = smp_call_function_single(cmd.cpuid, sgxvec_enable_apic, NULL, 1);
    if (!rc) {
      clear_bit(cmd.cpuid, g_apic_disabled_mask);
    }
    break;
  case SGXVEC_RESET_ALL_CMD:
    sgxvec_enable_apic_all();
    rc = 0;
    break;
  default:
    rc = -EINVAL;
    break;
  }
  mutex_unlock(&g_sgxvec_lock);

  if (rc) {
    return rc;
  }
  return (ssize_t)count;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static const struct proc_ops sgxvec_proc_ops = {
    .proc_write = sgxvec_proc_write,
};
#else
static const struct file_operations sgxvec_proc_ops = {
    .owner = THIS_MODULE,
    .write = sgxvec_proc_write,
};
#endif

static int __init sgxvec_init(void) {
  struct proc_dir_entry *ent;

  ent = proc_create("sgxvec", 0666, NULL, &sgxvec_proc_ops);
  if (!ent) {
    return -ENOMEM;
  }

  pr_info("sgxvec: loaded (/proc/sgxvec)\n");
  return 0;
}

static void __exit sgxvec_exit(void) {
  mutex_lock(&g_sgxvec_lock);
  sgxvec_enable_apic_all();
  mutex_unlock(&g_sgxvec_lock);

  remove_proc_entry("sgxvec", NULL);
  pr_info("sgxvec: unloaded\n");
}

module_init(sgxvec_init);
module_exit(sgxvec_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SGXVEC: local APIC control for IFEW experiments");
