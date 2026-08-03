/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Compile-time compatibility shim between NuttX libc headers and AP_HAL.
 */

#pragma once

#include <unistd.h>

/* NuttX exposes getpagesize as a function-like macro.  AP_HAL::Flash uses the
 * same identifier as a virtual method, so prevent the macro from rewriting
 * that interface after unistd.h has established its include guard. */
#ifdef getpagesize
#undef getpagesize
#endif
