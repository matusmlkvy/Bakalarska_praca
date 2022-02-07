#ifndef EPUCK_DEFS_
#define EPUCK_DEFS_


// basic defines for DLL export
#if (defined WIN32 || defined _WIN32 || defined WINCE || defined __CYGWIN__)
#	ifdef EPUCK_DLL_EXPORT
#		define EPUCK_API	__declspec(dllexport)
#	else
#		define EPUCK_API	__declspec(dllimport) 
#	endif
#elif defined __GNUC__ && __GNUC__ >= 4
#	define EPUCK_API		__attribute__ ((visibility ("default")))
#else
#	define EPUCK_API
#endif

#endif
