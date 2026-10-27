-- ============================================================================
-- lsyncd: 本机 → RC 单向实时同步 (legged)
--   启动:  lsyncd ~/legged_control/lsyncd_rc.lua            (后台)
--          lsyncd -nodaemon ~/legged_control/lsyncd_rc.lua  (前台调试)
--   停止:  pkill lsyncd
--   日志:  tail -f /tmp/lsyncd_rc.log
-- ============================================================================

settings {
    logfile        = "/tmp/lsyncd_rc.log",
    statusFile     = "/tmp/lsyncd_rc.status",
    statusInterval = 5,
    nodaemon       = false,
    inotifyMode    = "CloseWrite",
    maxProcesses   = 1,
    insist         = true,
}

-- ============================================================================
-- 共享配置
-- ============================================================================
local COMMON_EXCLUDE = {
    "build/",
    "install/",
    "log/",
    ".cache/",
    ".colcon/",
    "__pycache__/",
    "*.pyc",
    "*.so",
    "*.egg-info/",
    ".git/",
    ".vscode/",
    ".idea/",
    "*.swp",
    "*.swo",
    "*~",
    "compile_commands.json",
    "*.bag",
    "*.bag.active",
    "*.db3",
    "*.db3-wal",
    "*.db3-shm",
}

local COMMON_RSYNC = {
    archive    = true,
    compress   = true,
    _extra     = {
        "--delete",
        "--info=stats0",
        "--no-perms",
        "--no-owner",
        "--no-group",
        "--omit-dir-times",
    },
}

local COMMON_SSH = {
    port   = 22,
    _extra = {
        "-o", "ServerAliveInterval=15",
        "-o", "ServerAliveCountMax=3",
        "-o", "ConnectTimeout=5",
        "-o", "BatchMode=yes",
    },
}

-- ============================================================================
-- legged
-- ============================================================================
sync {
    default.rsyncssh,

    source    = "/home/rum/legged_control/legged/",
    host      = "rc@RC.local",
    targetdir = "/home/rc/Desktop/legged/",

    delay     = 1,

    exclude = COMMON_EXCLUDE,
    rsync   = COMMON_RSYNC,
    ssh     = COMMON_SSH,
}

