#include <iostream>
#include <cstdio>

// 对于 Windows 系统
#ifdef _WIN32
#include <windows.h>
#include <psapi.h>

void PrintMemoryInfo() {
    PROCESS_MEMORY_COUNTERS pmc;
    HANDLE hProcess = GetCurrentProcess();
    if (GetProcessMemoryInfo(hProcess, &pmc, sizeof(pmc))) {
        printf("PageFaultCount: %u\n", pmc.PageFaultCount);
        printf("PeakWorkingSetSize: %u\n", (unsigned int)pmc.PeakWorkingSetSize);
        printf("WorkingSetSize: %u\n", (unsigned int)pmc.WorkingSetSize);
        // 添加更多打印信息...
    }
}

// 对于 macOS 系统
#elif defined(__APPLE__)
#include <mach/mach.h>

void PrintMemoryInfo() {
    struct mach_task_basic_info info;
    mach_msg_type_number_t infoCount = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO, (task_info_t)&info, &infoCount) == KERN_SUCCESS) {
        printf("Virtual Size: %llu MB\n", info.virtual_size / 1024 / 1024);
        printf("Resident Size: %llu MB\n", info.resident_size / 1024 / 1024);
        // 添加更多打印信息...
    }
}

// 如果不是上述两个系统，则不做任何事
#else
void PrintMemoryInfo() {
    printf("Unsupported platform.\n");
}
#endif

int main(int argc, char *argv[]) {
    PrintMemoryInfo();
    return 0;
}

