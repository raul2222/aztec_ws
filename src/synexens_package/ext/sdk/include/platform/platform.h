#ifndef LIBSYNEXENS3_PLATFORM_PLATFORM_H
#define LIBSYNEXENS3_PLATFORM_PLATFORM_H
#include<map>


#if defined(_MSC_VER)


#else

#endif

#include"platform-interface.h"


namespace SY3_NAMESPACE
{

    namespace platform
    {
        const uint16_t CS30_VID = 0x2207;
        const uint16_t CS30_PID = 0x0016;

		const uint16_t CS20_VID = 0x2222;
		const uint16_t CS20_DUAL_FREQ_PID = 0x6666;
        const uint16_t CS20_SINGLE_FREQ_PID = 0x7777;

	    int verification_validity();

		int find_target_device_vid_pid(uint16_t &vid, uint16_t &pid);

        platform_interface* create_platform_device(uint16_t vid, uint16_t pid);

    };
};

#endif