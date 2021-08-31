#ifdef _USE_OPENCV
#include <opencv2/core.hpp>
#endif
#include "ARInterface.h"
#include "ARInterface_impl.hpp"
template DLL_API bool MVSA::ARCHIVE::SerializeSave<MVSA::Interface>(
    const MVSA::Interface &, const std::string &, int, uint32_t);

template DLL_API bool MVSA::ARCHIVE::SerializeLoad<MVSA::Interface>(
    MVSA::Interface &,
    const std::string &,
    int* , uint32_t *);
