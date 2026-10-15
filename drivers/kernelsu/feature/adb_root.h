#ifndef __KSU_H_ADB_ROOT
#define __KSU_H_ADB_ROOT
#include <asm/ptrace.h>

long ksu_adb_root_handle_execve(struct pt_regs *regs);
long ksu_adb_root_handle_execveat(struct pt_regs *regs);

/*
 * TB301FU: returns 1 when filename_user points at adbd. Used by
 * hook_manager.c to decide whether the adb_root handler consumed the
 * execve; when it did not, sucompat must still get its chance.
 */
long is_exec_adbd(const char __user *filename_user);

void ksu_adb_root_init(void);

void ksu_adb_root_exit(void);

#endif