/*
 * LD_PRELOAD shim that remaps libwebkit2gtk-4.1.so.0's hardcoded
 * helper-process path to the AppImage's bundled equivalent.
 *
 * Background: webkit2gtk's main library spawns a few sibling helper
 * binaries (WebKitNetworkProcess, WebKitWebProcess, WebKitGPUProcess,
 * MiniBrowser, injected-bundle) at runtime. The path it looks for them
 * at is the CMake PKGLIBEXECDIR baked in at build time — for Ubuntu's
 * packaging, that's "/usr/lib/x86_64-linux-gnu/webkit2gtk-4.1/".
 *
 * webkit2gtk does have a WEBKIT_EXEC_PATH env var that overrides this,
 * but it's gated behind ENABLE(DEVELOPER_MODE) in the CMake config —
 * which distro builds turn OFF, compiling out the env-var lookup
 * entirely. So that path is hardcoded into the .so file we bundle,
 * and the Deck (Arch-based, doesn't even have /usr/lib/x86_64-linux-
 * gnu/) can't satisfy it.
 *
 * Solution: intercept the syscalls webkit uses to launch the helpers
 * (execve + posix_spawn family, plus open/access for any pre-launch
 * existence checks), spot any path starting with the hardcoded prefix,
 * and rewrite it to point inside the AppImage's mount where we bundled
 * the helpers. The remapping uses the APPDIR env var which AppImages
 * set when the runtime mounts the squashfs at /tmp/.mount_<random>/.
 *
 * Only the webkit helper path is remapped — everything else passes
 * through unchanged. The shim is loaded via LD_PRELOAD by the AppRun
 * script before exec'ing saint-controller.
 *
 * Compile with:
 *   gcc -shared -fPIC -o libpath-shim.so path-shim.c -ldl
 */

#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <spawn.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static const char WEBKIT_HOST_PREFIX[] = "/usr/lib/x86_64-linux-gnu/webkit2gtk-4.1/";
#define HOST_PREFIX_LEN (sizeof(WEBKIT_HOST_PREFIX) - 1)

/*
 * Returns a malloc'd remapped path if `path` starts with the webkit
 * host prefix and APPDIR is set; NULL otherwise. Caller frees.
 */
static char *remap_path(const char *path)
{
    if (!path)
        return NULL;
    if (strncmp(path, WEBKIT_HOST_PREFIX, HOST_PREFIX_LEN) != 0)
        return NULL;

    const char *appdir = getenv("APPDIR");
    if (!appdir || !*appdir)
        return NULL;

    size_t needed = strlen(appdir) + strlen(path) + 1;
    char *out = malloc(needed);
    if (!out)
        return NULL;
    /* APPDIR + "/usr/lib/x86_64-linux-gnu/webkit2gtk-4.1/<rest>" */
    snprintf(out, needed, "%s%s", appdir, path);
    return out;
}

/*
 * exec*() only returns to the caller on failure (success replaces the
 * process image, never returns). The free() on the remapped path is
 * effectively only relevant in the failure case, but harmless either
 * way since the process is gone before the leak would matter.
 */
int execve(const char *path, char *const argv[], char *const envp[])
{
    typedef int (*fn_t)(const char *, char *const[], char *const[]);
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "execve");
    char *remapped = remap_path(path);
    const char *use = remapped ? remapped : path;
    int ret = real(use, argv, envp);
    free(remapped);
    return ret;
}

int execvp(const char *file, char *const argv[])
{
    typedef int (*fn_t)(const char *, char *const[]);
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "execvp");
    char *remapped = remap_path(file);
    const char *use = remapped ? remapped : file;
    int ret = real(use, argv);
    free(remapped);
    return ret;
}

int execvpe(const char *file, char *const argv[], char *const envp[])
{
    typedef int (*fn_t)(const char *, char *const[], char *const[]);
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "execvpe");
    char *remapped = remap_path(file);
    const char *use = remapped ? remapped : file;
    int ret = real(use, argv, envp);
    free(remapped);
    return ret;
}

int posix_spawn(pid_t *pid, const char *path,
                const posix_spawn_file_actions_t *file_actions,
                const posix_spawnattr_t *attrp,
                char *const argv[], char *const envp[])
{
    typedef int (*fn_t)(pid_t *, const char *,
                        const posix_spawn_file_actions_t *,
                        const posix_spawnattr_t *,
                        char *const[], char *const[]);
    char *remapped = remap_path(path);
    const char *use = remapped ? remapped : path;
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "posix_spawn");
    int ret = real(pid, use, file_actions, attrp, argv, envp);
    free(remapped);
    return ret;
}

int posix_spawnp(pid_t *pid, const char *file,
                 const posix_spawn_file_actions_t *file_actions,
                 const posix_spawnattr_t *attrp,
                 char *const argv[], char *const envp[])
{
    typedef int (*fn_t)(pid_t *, const char *,
                        const posix_spawn_file_actions_t *,
                        const posix_spawnattr_t *,
                        char *const[], char *const[]);
    char *remapped = remap_path(file);
    const char *use = remapped ? remapped : file;
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "posix_spawnp");
    int ret = real(pid, use, file_actions, attrp, argv, envp);
    free(remapped);
    return ret;
}

int access(const char *pathname, int mode)
{
    typedef int (*fn_t)(const char *, int);
    char *remapped = remap_path(pathname);
    const char *use = remapped ? remapped : pathname;
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "access");
    int ret = real(use, mode);
    free(remapped);
    return ret;
}

int faccessat(int dirfd, const char *pathname, int mode, int flags)
{
    typedef int (*fn_t)(int, const char *, int, int);
    char *remapped = remap_path(pathname);
    const char *use = remapped ? remapped : pathname;
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "faccessat");
    int ret = real(dirfd, use, mode, flags);
    free(remapped);
    return ret;
}

int open(const char *pathname, int flags, ...)
{
    typedef int (*fn_t)(const char *, int, ...);
    mode_t mode = 0;
    /* O_CREAT or O_TMPFILE indicates a mode argument was passed.
     * O_TMPFILE is a Linux extension visible only under _GNU_SOURCE;
     * guarded so a stricter libc that lacks it still compiles. */
    int needs_mode = (flags & O_CREAT) != 0;
#ifdef O_TMPFILE
    needs_mode = needs_mode || (flags & O_TMPFILE) != 0;
#endif
    if (needs_mode) {
        va_list ap;
        va_start(ap, flags);
        /* Read as int — mode_t is promotion-eligible (unsigned short
         * on some ABIs), and va_arg on a promotable type is UB. */
        mode = (mode_t)va_arg(ap, int);
        va_end(ap);
    }
    char *remapped = remap_path(pathname);
    const char *use = remapped ? remapped : pathname;
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "open");
    int ret = real(use, flags, mode);
    free(remapped);
    return ret;
}

int open64(const char *pathname, int flags, ...)
{
    typedef int (*fn_t)(const char *, int, ...);
    mode_t mode = 0;
    /* O_CREAT or O_TMPFILE indicates a mode argument was passed.
     * O_TMPFILE is a Linux extension visible only under _GNU_SOURCE;
     * guarded so a stricter libc that lacks it still compiles. */
    int needs_mode = (flags & O_CREAT) != 0;
#ifdef O_TMPFILE
    needs_mode = needs_mode || (flags & O_TMPFILE) != 0;
#endif
    if (needs_mode) {
        va_list ap;
        va_start(ap, flags);
        /* Read as int — mode_t is promotion-eligible (unsigned short
         * on some ABIs), and va_arg on a promotable type is UB. */
        mode = (mode_t)va_arg(ap, int);
        va_end(ap);
    }
    char *remapped = remap_path(pathname);
    const char *use = remapped ? remapped : pathname;
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "open64");
    int ret = real(use, flags, mode);
    free(remapped);
    return ret;
}

int openat(int dirfd, const char *pathname, int flags, ...)
{
    typedef int (*fn_t)(int, const char *, int, ...);
    mode_t mode = 0;
    /* O_CREAT or O_TMPFILE indicates a mode argument was passed.
     * O_TMPFILE is a Linux extension visible only under _GNU_SOURCE;
     * guarded so a stricter libc that lacks it still compiles. */
    int needs_mode = (flags & O_CREAT) != 0;
#ifdef O_TMPFILE
    needs_mode = needs_mode || (flags & O_TMPFILE) != 0;
#endif
    if (needs_mode) {
        va_list ap;
        va_start(ap, flags);
        /* Read as int — mode_t is promotion-eligible (unsigned short
         * on some ABIs), and va_arg on a promotable type is UB. */
        mode = (mode_t)va_arg(ap, int);
        va_end(ap);
    }
    char *remapped = remap_path(pathname);
    const char *use = remapped ? remapped : pathname;
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "openat");
    int ret = real(dirfd, use, flags, mode);
    free(remapped);
    return ret;
}

int stat(const char *pathname, struct stat *statbuf)
{
    typedef int (*fn_t)(const char *, struct stat *);
    char *remapped = remap_path(pathname);
    const char *use = remapped ? remapped : pathname;
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "stat");
    int ret = real(use, statbuf);
    free(remapped);
    return ret;
}

int lstat(const char *pathname, struct stat *statbuf)
{
    typedef int (*fn_t)(const char *, struct stat *);
    char *remapped = remap_path(pathname);
    const char *use = remapped ? remapped : pathname;
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "lstat");
    int ret = real(use, statbuf);
    free(remapped);
    return ret;
}

int fstatat(int dirfd, const char *pathname, struct stat *statbuf, int flags)
{
    typedef int (*fn_t)(int, const char *, struct stat *, int);
    char *remapped = remap_path(pathname);
    const char *use = remapped ? remapped : pathname;
    static fn_t real = NULL;
    if (!real)
        real = (fn_t)dlsym(RTLD_NEXT, "fstatat");
    int ret = real(dirfd, use, statbuf, flags);
    free(remapped);
    return ret;
}
