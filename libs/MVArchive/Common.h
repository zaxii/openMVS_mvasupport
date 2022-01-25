//
// Created by song.1634 on 1/25/2022.
//

#ifndef OPENMVS_COMMON_H
#define OPENMVS_COMMON_H


// I N C L U D E S /////////////////////////////////////////////////

#if defined(Mva_EXPORTS) && !defined(Common_EXPORTS)
#define Common_EXPORTS
#endif

#include "../Common/Common.h"

#ifndef MVS_API
#define MVS_API GENERAL_API
#endif
#ifndef MVS_TPL
#define MVS_TPL GENERAL_TPL
#endif

#endif //OPENMVS_COMMON_H
