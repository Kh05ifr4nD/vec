#include <linux/capability.h>
#include <linux/cred.h>
#include <linux/cpu.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>

#define SGXVEC_SCHEMA_VERSION 1
#define SGXVEC_MAX_CMD 512
#define SGXVEC_MAX_HISTORY 256

static char *win_cpulist = "";
static char *hk_cpulist = "";
static int sample_period_ms = 1000;
static int sample_history = 32;

module_param(win_cpulist, charp, 0644);
MODULE_PARM_DESC(win_cpulist, "window CPU list (cpulist format, e.g. 2-15)");
module_param(hk_cpulist, charp, 0644);
MODULE_PARM_DESC(hk_cpulist, "housekeeping CPU list (cpulist format, e.g. 0-1)");
module_param(sample_period_ms, int, 0644);
MODULE_PARM_DESC(sample_period_ms, "sampling period in ms (0 disables sampling)");
module_param(sample_history, int, 0644);
MODULE_PARM_DESC(sample_history, "number of samples to keep (ring buffer)");

struct sgxvec_sample {
  u64 ts_ns;
  u64 win_irq_total;
  u64 win_irq_delta;
};

struct sgxvec_migrate_summary {
  u64 ts_ns;
  unsigned int attempted;
  unsigned int ok;
  unsigned int skipped;
  unsigned int failed;
  unsigned int fail_eperm;
  unsigned int fail_other;
};

static DEFINE_MUTEX(g_sgxvec_lock);
static struct proc_dir_entry *g_proc_ent;
static struct delayed_work g_sample_work;

static cpumask_var_t g_win_mask;
static cpumask_var_t g_hk_mask;

static struct sgxvec_sample *g_samples;
static unsigned int g_samples_cap;
static unsigned int g_samples_head;
static bool g_samples_filled;
static u64 g_last_win_irq_total;

static struct sgxvec_migrate_summary g_last_migrate;

static u64 sgxvec_now_ns(void) {
  struct timespec64 ts;
  ktime_get_real_ts64(&ts);
  return (u64)ts.tv_sec * 1000000000ull + (u64)ts.tv_nsec;
}

static u64 sgxvec_irq_total_for_cpu(int cpu) {
  return (u64)kstat_cpu_irqs_sum((unsigned int)cpu);
}

static u64 sgxvec_irq_total_for_mask(const struct cpumask *mask) {
  u64 sum = 0;
  int cpu;
  for_each_cpu(cpu, mask) {
    if (!cpu_online(cpu))
      continue;
    sum += sgxvec_irq_total_for_cpu(cpu);
  }
  return sum;
}

static void sgxvec_clear_samples_locked(void) {
  g_samples_head = 0;
  g_samples_filled = false;
  g_last_win_irq_total = 0;
  if (g_samples && g_samples_cap) {
    memset(g_samples, 0, sizeof(*g_samples) * g_samples_cap);
  }
}

static void sgxvec_schedule_next_sample_locked(void) {
  if (sample_period_ms <= 0)
    return;
  if (!g_win_mask || cpumask_empty(g_win_mask))
    return;
  schedule_delayed_work(&g_sample_work, msecs_to_jiffies(sample_period_ms));
}

static void sgxvec_sample_workfn(struct work_struct *work) {
  u64 now_ns;
  u64 total;
  u64 delta;

  (void)work;
  mutex_lock(&g_sgxvec_lock);
  if (!g_win_mask || cpumask_empty(g_win_mask)) {
    mutex_unlock(&g_sgxvec_lock);
    return;
  }
  now_ns = sgxvec_now_ns();
  total = sgxvec_irq_total_for_mask(g_win_mask);
  delta = (g_last_win_irq_total == 0) ? 0 : (total - g_last_win_irq_total);
  g_last_win_irq_total = total;

  if (g_samples && g_samples_cap) {
    g_samples[g_samples_head].ts_ns = now_ns;
    g_samples[g_samples_head].win_irq_total = total;
    g_samples[g_samples_head].win_irq_delta = delta;
    g_samples_head = (g_samples_head + 1) % g_samples_cap;
    if (g_samples_head == 0)
      g_samples_filled = true;
  }
  sgxvec_schedule_next_sample_locked();
  mutex_unlock(&g_sgxvec_lock);
}

static void sgxvec_seq_json_string(struct seq_file *m, const char *s) {
  const char *p = s ? s : "";
  seq_putc(m, '"');
  for (; *p; ++p) {
    const char c = *p;
    if (c == '\\' || c == '"') {
      seq_putc(m, '\\');
      seq_putc(m, c);
      continue;
    }
    if (c == '\n') {
      seq_puts(m, "\\\\n");
      continue;
    }
    seq_putc(m, c);
  }
  seq_putc(m, '"');
}

static void sgxvec_emit_verify(struct seq_file *m) {
  bool ok = true;
  unsigned int err_n = 0;

  seq_puts(m, "\"verify\":{");

  // Minimal, deterministic checks (non-TCB): configuration presence and sanity.
  if (!g_win_mask || cpumask_empty(g_win_mask)) {
    ok = false;
  }
  if (!g_hk_mask || cpumask_empty(g_hk_mask)) {
    ok = false;
  }
  if (g_win_mask && g_hk_mask && cpumask_intersects(g_win_mask, g_hk_mask)) {
    ok = false;
  }

  seq_printf(m, "\"ok\":%s,", ok ? "true" : "false");
  seq_puts(m, "\"errors\":[");
  if (!g_win_mask || cpumask_empty(g_win_mask)) {
    sgxvec_seq_json_string(m, "win_cpulist 未配置或为空");
    ++err_n;
  }
  if (!g_hk_mask || cpumask_empty(g_hk_mask)) {
    if (err_n)
      seq_putc(m, ',');
    sgxvec_seq_json_string(m, "hk_cpulist 未配置或为空");
    ++err_n;
  }
  if (g_win_mask && g_hk_mask && cpumask_intersects(g_win_mask, g_hk_mask)) {
    if (err_n)
      seq_putc(m, ',');
    sgxvec_seq_json_string(m, "win/hk 掩码重叠（必须互斥）");
    ++err_n;
  }
  seq_puts(m, "],");

  // Warnings: offline CPUs in masks (diagnostic).
  seq_puts(m, "\"warnings\":[");
  if (g_win_mask) {
    int cpu;
    bool first = true;
    for_each_cpu(cpu, g_win_mask) {
      if (cpu_online(cpu))
        continue;
      if (!first)
        seq_putc(m, ',');
      first = false;
      seq_printf(m, "\"win cpu%d offline\"", cpu);
    }
  }
  seq_puts(m, "]}");
}

static int sgxvec_proc_show(struct seq_file *m, void *v) {
  u64 now_ns;
  struct sgxvec_sample last = {0};
  bool have_last = false;

  (void)v;

  mutex_lock(&g_sgxvec_lock);
  now_ns = sgxvec_now_ns();

  if (g_samples && g_samples_cap && (g_samples_filled || g_samples_head > 0)) {
    unsigned int idx = (g_samples_head == 0) ? (g_samples_cap - 1) : (g_samples_head - 1);
    last = g_samples[idx];
    have_last = (last.ts_ns != 0);
  }

  seq_puts(m, "{");
  seq_printf(m, "\"schema_version\":%d,", SGXVEC_SCHEMA_VERSION);
  seq_printf(m, "\"generated_at_ns\":%llu,", (unsigned long long)now_ns);

  // config
  seq_puts(m, "\"config\":{");
  seq_puts(m, "\"win_cpulist\":");
  if (g_win_mask && !cpumask_empty(g_win_mask)) {
    seq_printf(m, "\"%*pbl\",", cpumask_pr_args(g_win_mask));
  } else {
    seq_puts(m, "\"\",");
  }
  seq_puts(m, "\"hk_cpulist\":");
  if (g_hk_mask && !cpumask_empty(g_hk_mask)) {
    seq_printf(m, "\"%*pbl\"", cpumask_pr_args(g_hk_mask));
  } else {
    seq_puts(m, "\"\"");
  }
  seq_puts(m, "},");

  // sampling
  seq_puts(m, "\"sample\":{");
  seq_printf(m, "\"period_ms\":%d,", sample_period_ms);
  seq_printf(m, "\"history\":%u,", g_samples_cap);
  seq_printf(m, "\"have_last\":%s,", have_last ? "true" : "false");
  seq_puts(m, "\"last\":{");
  seq_printf(m, "\"ts_ns\":%llu,", (unsigned long long)last.ts_ns);
  seq_printf(m, "\"win_irq_total\":%llu,", (unsigned long long)last.win_irq_total);
  seq_printf(m, "\"win_irq_delta\":%llu", (unsigned long long)last.win_irq_delta);
  seq_puts(m, "},");
  seq_puts(m, "\"nmi_supported\":false");
  seq_puts(m, "},");

  // migrate summary
  seq_puts(m, "\"migrate\":{");
  seq_printf(m, "\"ts_ns\":%llu,", (unsigned long long)g_last_migrate.ts_ns);
  seq_printf(m, "\"attempted\":%u,", g_last_migrate.attempted);
  seq_printf(m, "\"ok\":%u,", g_last_migrate.ok);
  seq_printf(m, "\"skipped\":%u,", g_last_migrate.skipped);
  seq_printf(m, "\"failed\":%u,", g_last_migrate.failed);
  seq_printf(m, "\"fail_eperm\":%u,", g_last_migrate.fail_eperm);
  seq_printf(m, "\"fail_other\":%u", g_last_migrate.fail_other);
  seq_puts(m, "},");

  sgxvec_emit_verify(m);

  seq_puts(m, "}\n");
  mutex_unlock(&g_sgxvec_lock);
  return 0;
}

static int sgxvec_proc_open(struct inode *inode, struct file *file) {
  return single_open(file, sgxvec_proc_show, NULL);
}

static int sgxvec_parse_kv(const char *tok, const char *key, const char **out) {
  size_t n;
  if (!tok || !key || !out)
    return 0;
  n = strlen(key);
  if (strncmp(tok, key, n) != 0)
    return 0;
  if (tok[n] != '=')
    return 0;
  *out = tok + n + 1;
  return 1;
}

static bool sgxvec_config_ok_locked(void) {
  if (!g_win_mask || cpumask_empty(g_win_mask))
    return false;
  if (!g_hk_mask || cpumask_empty(g_hk_mask))
    return false;
  if (cpumask_intersects(g_win_mask, g_hk_mask))
    return false;
  return true;
}

static int sgxvec_apply_config_locked(const char *win, const char *hk,
                                     bool require_complete) {
  cpumask_var_t win_tmp;
  cpumask_var_t hk_tmp;
  int rc;

  if (require_complete) {
    if (!win || !*win || !hk || !*hk)
      return -EINVAL;
  }

  if (!zalloc_cpumask_var(&win_tmp, GFP_KERNEL))
    return -ENOMEM;
  if (!zalloc_cpumask_var(&hk_tmp, GFP_KERNEL)) {
    free_cpumask_var(win_tmp);
    return -ENOMEM;
  }

  if (win && *win) {
    rc = cpulist_parse(win, win_tmp);
    if (rc) {
      free_cpumask_var(win_tmp);
      free_cpumask_var(hk_tmp);
      return rc;
    }
  }
  if (hk && *hk) {
    rc = cpulist_parse(hk, hk_tmp);
    if (rc) {
      free_cpumask_var(win_tmp);
      free_cpumask_var(hk_tmp);
      return rc;
    }
  }

  if (require_complete) {
    if (cpumask_empty(win_tmp) || cpumask_empty(hk_tmp) ||
        cpumask_intersects(win_tmp, hk_tmp)) {
      free_cpumask_var(win_tmp);
      free_cpumask_var(hk_tmp);
      return -EINVAL;
    }
  }

  if (g_win_mask)
    cpumask_copy(g_win_mask, win_tmp);
  if (g_hk_mask)
    cpumask_copy(g_hk_mask, hk_tmp);

  sgxvec_clear_samples_locked();
  sgxvec_schedule_next_sample_locked();

  free_cpumask_var(win_tmp);
  free_cpumask_var(hk_tmp);
  return 0;
}

static void sgxvec_migrate_irqs_locked(void) {
  cpumask_var_t target;
  unsigned int irq;

  memset(&g_last_migrate, 0, sizeof(g_last_migrate));
  g_last_migrate.ts_ns = sgxvec_now_ns();

  if (!sgxvec_config_ok_locked()) {
    g_last_migrate.fail_other = 1;
    g_last_migrate.failed = 1;
    return;
  }

  if (!zalloc_cpumask_var(&target, GFP_KERNEL)) {
    g_last_migrate.fail_other = 1;
    g_last_migrate.failed = 1;
    return;
  }

  cpumask_and(target, g_hk_mask, cpu_online_mask);
  if (cpumask_empty(target)) {
    free_cpumask_var(target);
    g_last_migrate.fail_other = 1;
    g_last_migrate.failed = 1;
    return;
  }

  for (irq = 0; irq < nr_irqs; ++irq) {
    int rc;
    ++g_last_migrate.attempted;
    rc = irq_set_affinity(irq, target);
    if (rc == 0) {
      ++g_last_migrate.ok;
      continue;
    }
    if (rc == -EINVAL || rc == -ENOSYS || rc == -ENXIO) {
      ++g_last_migrate.skipped;
      continue;
    }
    ++g_last_migrate.failed;
    if (rc == -EPERM) {
      ++g_last_migrate.fail_eperm;
    } else {
      ++g_last_migrate.fail_other;
    }
  }

  free_cpumask_var(target);
}

static ssize_t sgxvec_proc_write(struct file *file, const char __user *buffer,
                                 size_t count, loff_t *ppos) {
  char *kbuf;
  ssize_t rc = -EINVAL;
  const size_t n = (count > SGXVEC_MAX_CMD) ? SGXVEC_MAX_CMD : count;

  (void)file;
  (void)ppos;

  if (!capable(CAP_SYS_ADMIN)) {
    return -EPERM;
  }

  kbuf = kzalloc(n + 1, GFP_KERNEL);
  if (!kbuf)
    return -ENOMEM;
  if (copy_from_user(kbuf, buffer, n)) {
    kfree(kbuf);
    return -EFAULT;
  }
  kbuf[n] = '\0';

  // Trim trailing newline.
  if (n > 0 && kbuf[n - 1] == '\n')
    kbuf[n - 1] = '\0';

  mutex_lock(&g_sgxvec_lock);
  if (strncmp(kbuf, "configure", 9) == 0 &&
      (kbuf[9] == '\0' || kbuf[9] == ' ' || kbuf[9] == '\t')) {
    const char *win = NULL;
    const char *hk = NULL;
    char *s = kbuf + 9;
    char *tok;
    while (s && *s) {
      while (*s == ' ' || *s == '\t')
        ++s;
      tok = strsep(&s, " \t");
      if (!tok || !*tok)
        continue;
      (void)sgxvec_parse_kv(tok, "win", &win);
      (void)sgxvec_parse_kv(tok, "hk", &hk);
      (void)sgxvec_parse_kv(tok, "housekeeping", &hk);
    }
    mutex_unlock(&g_sgxvec_lock);
    cancel_delayed_work_sync(&g_sample_work);
    mutex_lock(&g_sgxvec_lock);
    rc = sgxvec_apply_config_locked(win, hk, true);
    if (rc == 0)
      rc = (ssize_t)count;
  } else if (strcmp(kbuf, "migrate_irqs") == 0) {
    sgxvec_migrate_irqs_locked();
    rc = (ssize_t)count;
  } else if (strcmp(kbuf, "clear") == 0) {
    if (g_win_mask)
      cpumask_clear(g_win_mask);
    if (g_hk_mask)
      cpumask_clear(g_hk_mask);
    sgxvec_clear_samples_locked();
    memset(&g_last_migrate, 0, sizeof(g_last_migrate));
    mutex_unlock(&g_sgxvec_lock);
    cancel_delayed_work_sync(&g_sample_work);
    mutex_lock(&g_sgxvec_lock);
    rc = (ssize_t)count;
  } else {
    rc = -EINVAL;
  }
  mutex_unlock(&g_sgxvec_lock);

  kfree(kbuf);
  return rc;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static const struct proc_ops sgxvec_proc_ops = {
    .proc_open = sgxvec_proc_open,
    .proc_read = seq_read,
    .proc_lseek = seq_lseek,
    .proc_release = single_release,
    .proc_write = sgxvec_proc_write,
};
#else
static const struct file_operations sgxvec_proc_ops = {
    .owner = THIS_MODULE,
    .open = sgxvec_proc_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
    .write = sgxvec_proc_write,
};
#endif

static int __init sgxvec_init(void) {
  int rc;
  int cap;

  if (!zalloc_cpumask_var(&g_win_mask, GFP_KERNEL))
    return -ENOMEM;
  if (!zalloc_cpumask_var(&g_hk_mask, GFP_KERNEL)) {
    free_cpumask_var(g_win_mask);
    return -ENOMEM;
  }

  cap = sample_history;
  if (cap <= 0)
    cap = 32;
  if (cap > SGXVEC_MAX_HISTORY)
    cap = SGXVEC_MAX_HISTORY;
  g_samples_cap = (unsigned int)cap;
  g_samples = kcalloc(g_samples_cap, sizeof(*g_samples), GFP_KERNEL);
  if (!g_samples) {
    free_cpumask_var(g_win_mask);
    free_cpumask_var(g_hk_mask);
    return -ENOMEM;
  }

  INIT_DELAYED_WORK(&g_sample_work, sgxvec_sample_workfn);

  mutex_lock(&g_sgxvec_lock);
  rc = sgxvec_apply_config_locked(win_cpulist, hk_cpulist, false);
  mutex_unlock(&g_sgxvec_lock);
  if (rc) {
    pr_warn("sgxvec: failed to parse module params (win=%s hk=%s rc=%d)\n", win_cpulist,
            hk_cpulist, rc);
    // Continue with empty config; status/verify will show non-ok.
  }

  g_proc_ent = proc_create("sgxvec", 0644, NULL, &sgxvec_proc_ops);
  if (!g_proc_ent) {
    cancel_delayed_work_sync(&g_sample_work);
    kfree(g_samples);
    free_cpumask_var(g_win_mask);
    free_cpumask_var(g_hk_mask);
    return -ENOMEM;
  }

  pr_info("sgxvec: loaded (/proc/sgxvec) - SAFE mode (no APIC register writes)\n");
  return 0;
}

static void __exit sgxvec_exit(void) {
  remove_proc_entry("sgxvec", NULL);
  cancel_delayed_work_sync(&g_sample_work);
  kfree(g_samples);
  free_cpumask_var(g_win_mask);
  free_cpumask_var(g_hk_mask);
  pr_info("sgxvec: unloaded\n");
}

module_init(sgxvec_init);
module_exit(sgxvec_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("sgxvec: guard/verify/irq-migrate helper (non-TCB; no APIC ops)");
