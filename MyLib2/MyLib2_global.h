#ifndef MYLIB2_GLOBAL_H
#define MYLIB2_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(MYLIB2_LIBRARY)
#define MYLIB2_EXPORT Q_DECL_EXPORT
#else
#define MYLIB2_EXPORT Q_DECL_IMPORT
#endif

#endif // MYLIB2_GLOBAL_H
